package org.sciborgs1155.robot.climb;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.shooter.ShooterConstants.MAX_VELOCITY;
import static org.sciborgs1155.robot.shooter.ShooterConstants.VELOCITY_TOLERANCE;

import org.sciborgs1155.robot.Robot;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;



public class Climb implements AutoCloseable {
    private final ClimbIO hardware;
    private final ProfiledPIDController controller = new ProfiledPIDController(
        ClimbConstants.P, 
        ClimbConstants.I, 
        ClimbConstants.D, 
        new TrapezoidProfile.Constraints(
            ClimbConstants.MAX_VELOCITY.in(MetersPerSecond), 
            ClimbConstants.MAX_ACCEL.in(MetersPerSecondPerSecond))); //for now 
        
    //elvator ff to take in cosideration gravity
    private final ElevatorFeedforward ff = new ElevatorFeedforward(0, 0, 0); //for now
    private final SysIdRoutine characterization;


    public static Climb create() {
        return Robot.isReal() ? new Climb(new RealClimb()) : new Climb(new SimClimb());
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


    

