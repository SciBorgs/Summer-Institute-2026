package org.sciborgs1155.robot.hood;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Constants.TUNING;
import static org.sciborgs1155.robot.hood.HoodConstants.*;
import static org.sciborgs1155.robot.hood.HoodConstants.PID.*;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.Robot;

/** adjust vertical shooting angle of fuel */
@Logged
public class Hood extends SubsystemBase implements AutoCloseable{
    private final HoodIO hardware;

    @Logged 
    private final ProfiledPIDController fb = 
        new ProfiledPIDController(P, I, D, 
            new TrapezoidProfile.Constraints(
                MAX_VELOCITY.in(RadiansPerSecond), MAX_ACCEL.in(RadiansPerSecondPerSecond))
        );
    
    private final ArmFeedforward ff = new ArmFeedforward(S, G, V, A);

    @NotLogged private final DoubleEntry tuningP = Tuning.entry("Robot/tuning/hood/K_P", P);
    @NotLogged private final DoubleEntry tuningI = Tuning.entry("Robot/tuning/hood/K_I", I);
    @NotLogged private final DoubleEntry tuningD = Tuning.entry("Robot/tuning/hood/K_D", D);
    @NotLogged private final DoubleEntry tuningS = Tuning.entry("Robot/tuning/hood/S", S);
    @NotLogged private final DoubleEntry tuningG = Tuning.entry("Robot/tuning/hood/G", G);
    @NotLogged private final DoubleEntry tuningV = Tuning.entry("Robot/tuning/hood/V", V);
    @NotLogged private final DoubleEntry tuningA = Tuning.entry("Robot/tuning/hood/A", A);

    /** routine for recording/analyzing motor data */
    private final SysIdRoutine sysIdRoutine;

    /**
     * 
     * @return a real or sim hood subsysyem
     */
    public static Hood create(){
        return new Hood(Robot.isReal() ? new RealHood() : new SimHood());
    }

    /**
     * @return hood with no interface
     */
    public static Hood none(){
        return new Hood(new NoHood());
    }

    /**
     * CONSTRUCTOR 
     * @param hardware : hoodIO object that will be operated on
     */
    private Hood(HoodIO hardware){
        this.hardware = hardware;

        fb.setTolerance(POSITION_TOLERANCE.in(Radians));
        fb.reset(angle());
        setDefaultCommand(goTo(DEFAULT_ANGLE));

        sysIdRoutine = 
            new SysIdRoutine(
                new Config(RAMP_RATE, STEP_VOLTAGE, TIME_OUT), 
                new Mechanism(voltage -> hardware.setVoltage(voltage.in(Volts)), null, this)
            );
    
        SmartDashboard.putData(
                "Robot/hood/quasistatic forward",
                sysIdRoutine
                    .quasistatic(Direction.kForward)
                    .until(() -> atPosition(MAX_ANGLE.in(Radians)))
                    .withName("hood quasistatic forward"));
        SmartDashboard.putData(
                "Robot/hood/quasistatic backward",
                sysIdRoutine
                    .quasistatic(Direction.kReverse)
                    .until(() -> atPosition(MIN_ANGLE.in(Radians)))
                    .withName("hood quasistatic backward"));
        SmartDashboard.putData(
                "Robot/hood/dynamic forward",
                sysIdRoutine
                    .dynamic(Direction.kForward)
                    .until(() -> atPosition(MAX_ANGLE.in(Radians)))
                    .withName("hood dynamic forward"));
        SmartDashboard.putData(
                "Robot/hood/dynamic backward",
                sysIdRoutine
                    .dynamic(Direction.kReverse)
                    .until(() -> atPosition(MIN_ANGLE.in(Radians)))
                    .withName("hood dynamic backward"));
    }

    /**
     * @return current angle in radians
     */
    @Logged
    public double angle(){
        return hardware.angle();
    }
    
    /**
     * @return angle setpoint of the hood
     */
    @Logged
    public double angleSetpoint(){
        return fb.getSetpoint().position;
    }

    /**
     * @return current velocity og hood
     */
    @Logged
    public double velocity(){
        return hardware.velocity();
    }
    /**
     * @return velocity setpoint of hood
     */
    @Logged
    public double velocitySetpoind(){
        return fb.getSetpoint().velocity;
    }

    /**
     * @return whether hood is in desired state
     */
    public boolean atGoal(){
        return fb.atGoal();
    }

    /**
     * checks if hood is at certain position within tolerance
     * 
     * @param angle given position to check position to 
     * @return the check
     */
    public boolean atPosition(double angle){
        return Math.abs(angle - angle()) < POSITION_TOLERANCE.in(Radians);
    }

    /**
     * moves hood to goal angle
     * 
     * @params goal
     * @return a goTo command to set the hood to goal angle
     */
    public Command goTo(Angle goal){
        return goTo(() -> goal.in(Radians));
    }

    /** makes hood go to a set goal position */
    public Command goTo(DoubleSupplier goal){
        return run(() -> update(goal.getAsDouble())).withName("Hood GoTo");
    }

    /**
     * @param angle to shoot at
     * @param a command to go to shooting angle
     */
    public Command goToShootingAngle(DoubleSupplier goal){
        return run(() -> update(goal.getAsDouble() - SHOOTING_ANGLE_OFFSET.in(Radians)))
        .until(this::atGoal)
        .withName("Hood GoToAngle");
    }
    
    /**
     * 
     * @param goal angle to shoot at
     * @return a command to go to the shooting angle
     */
    public Command goToShootingAngle(Angle goal){
        return goToShootingAngle(() -> goal.in(Radians));
    }

    /**
     * method to set voltage of the motor based off ff and fb calculations
     * 
     * @param position goal angle 
     */
    private void update(double position){
        double goal = MathUtil.clamp(position, MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians));
        double feedback = fb.calculate(angle(), goal);
        double feedforward = ff.calculate(fb.getSetpoint().position, fb.getSetpoint().velocity);
        hardware.setVoltage(feedback + feedforward);
    }

    @Override
    public void close() throws Exception {
        hardware.close();
    }

  @Override
  public void periodic() {
    if (TUNING) {
      fb.setP(tuningP.get());
      fb.setI(tuningI.get());
      fb.setD(tuningD.get());
      ff.setKs(tuningS.get());
      ff.setKg(tuningG.get());
      ff.setKv(tuningV.get());
      ff.setKa(tuningA.get());
    }
    LoggingUtils.log(
        "/Robot/hood/command",
        Optional.ofNullable(getCurrentCommand()).map(Command::getName).orElse("none"));
  }
    
}