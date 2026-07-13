package org.sciborgs1155.robot.climb;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.shooter.ShooterConstants.VELOCITY_TOLERANCE;

import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.Robot;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;



public class Climb extends SubsystemBase implements AutoCloseable {
    private final ClimbIO hardware;
    private final ProfiledPIDController controller = new ProfiledPIDController(
        ClimbConstants.P, 
        ClimbConstants.I, 
        ClimbConstants.D, 
        new TrapezoidProfile.Constraints(
            ClimbConstants.MAX_VELOCITY.in(MetersPerSecond), 
            ClimbConstants.MAX_ACCEL.in(MetersPerSecondPerSecond))); //for now 
        
    //elvator ff to take in cosideration gravity
    private final ElevatorFeedforward ff = new ElevatorFeedforward(ClimbConstants.S, ClimbConstants.G, ClimbConstants.V, ClimbConstants.A); //for now
    private final SysIdRoutine characterization;


    /* When you suimulate, you can change the values in sumulation */
    private final DoubleEntry kS = Tuning.entry("/Robot/tuning/elevator/kS", ClimbConstants.S);
    private final DoubleEntry kG = Tuning.entry("/Robot/tuning/elevator/kG", ClimbConstants.G);
    private final DoubleEntry kV = Tuning.entry("/Robot/tuning/elevator/kV", ClimbConstants.V);
    private final DoubleEntry kA = Tuning.entry("/Robot/tuning/elevator/kA", ClimbConstants.A);

    public static Climb create() {
        return Robot.isReal() ? new Climb(new RealClimb()) : new Climb(new SimClimb());
    }

     public static Climb none() {
        return new Climb(new NoClimb());
     }

    public Climb(ClimbIO hardware) {
        this.hardware = hardware;

        controller.setTolerance(VELOCITY_TOLERANCE.in(RadiansPerSecond));//configure 
        controller.reset(hardware.getPosition());
        controller.setGoal(ClimbConstants.MIN_HEIGHT.in(Meters));

        characterization = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.per(Second).of(1), Volts.of(10.0), Seconds.of(11)),
            new SysIdRoutine.Mechanism(
                v ->  hardware.setVoltage(v.in(Volts)), null, (Subsystem) this, "climb"));

        //to do add if (TUNING)
        SmartDashboard.putData(
            "clibmb top quasistatic backward", characterization.quasistatic(Direction.kReverse));
        SmartDashboard.putData(
            "climb top quasistatic forward", characterization.quasistatic(Direction.kForward));
        SmartDashboard.putData(
            "Climb top dynamic backward", characterization.dynamic(Direction.kReverse));
        SmartDashboard.putData(
            "Climb top dynmaic forward", characterization.dynamic(Direction.kForward));
        
    }

    @Override
    public void close() throws Exception {
        hardware.close();
    }



}


    

