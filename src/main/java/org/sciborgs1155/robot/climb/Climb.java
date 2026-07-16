package org.sciborgs1155.robot.climb;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Constants.TUNING;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.Robot;

public class Climb extends SubsystemBase implements AutoCloseable {
  private final ClimbIO hardware;
  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          ClimbConstants.P,
          ClimbConstants.I,
          ClimbConstants.D,
          new TrapezoidProfile.Constraints(
              ClimbConstants.MAX_VELOCITY.in(MetersPerSecond),
              ClimbConstants.MAX_ACCEL.in(MetersPerSecondPerSecond))); // for now

  // elvator ff to take in cosideration gravity
  private final ElevatorFeedforward ff =
      new ElevatorFeedforward(
          ClimbConstants.S, ClimbConstants.G, ClimbConstants.V, ClimbConstants.A); // for now
  private final SysIdRoutine characterization;
  private final ClimbVisualizer climbVisualizer =
      new ClimbVisualizer("climb visualier", new Color8Bit(0, 0, 225));

  /* When you simulate, you can change the values in sumulation */
  private final DoubleEntry kS = Tuning.entry("/Robot/tuning/elevator/kS", ClimbConstants.S);
  private final DoubleEntry kG = Tuning.entry("/Robot/tuning/elevator/kG", ClimbConstants.G);
  private final DoubleEntry kV = Tuning.entry("/Robot/tuning/elevator/kV", ClimbConstants.V);
  private final DoubleEntry kA = Tuning.entry("/Robot/tuning/elevator/kA", ClimbConstants.A);

  /**
   * @return either RealClimb (with hardware) or SimClimb if Robot is real
   */
  public static Climb create() {
    return Robot.isReal() ? new Climb(new RealClimb()) : new Climb(new SimClimb());
  }

  /**
   * @return a climb without hardware (NoClimb)
   */
  public static Climb none() {
    return new Climb(new NoClimb());
  }

  public Climb(ClimbIO hardware) {
    this.hardware = hardware;

    controller.setTolerance(ClimbConstants.POSITION_TOLERANCE.in(Meters)); // configure
    controller.reset(hardware.getPosition());
    controller.setGoal(ClimbConstants.MIN_HEIGHT.in(Meters));

    characterization =
        new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(10.0), null),
            new SysIdRoutine.Mechanism(
                v -> hardware.setVoltage(v.in(Volts)), null, (Subsystem) this, "climb"));

    /* Tuning ensure it only works in test  */
    if (TUNING) {
      SmartDashboard.putData(
          "clibmb top quasistatic backward",
          characterization
              .quasistatic(Direction.kReverse)
              .until(() -> atPosition(ClimbConstants.MAX_HEIGHT.in(Meters))));
      SmartDashboard.putData(
          "climb top quasistatic forward",
          characterization
              .quasistatic(Direction.kForward)
              .until(() -> atPosition(ClimbConstants.MIN_HEIGHT.in(Meters) + 0.1)));
      SmartDashboard.putData(
          "Climb top dynamic backward",
          characterization
              .dynamic(Direction.kReverse)
              .until(() -> atPosition(ClimbConstants.MAX_HEIGHT.in(Meters))));
      SmartDashboard.putData(
          "Climb top dynmaic forward",
          characterization
              .dynamic(Direction.kForward)
              .until(() -> atPosition(ClimbConstants.MIN_HEIGHT.in(Meters) + 0.1)));
    }
  }

  /**
   * Checks if position is within the tolerance margin
   *
   * @param goal The desired goal in meters
   * @return Boolean depending on if it is within the tolerance
   */
  public Boolean atPosition(double goal) {
    return Math.abs(goal - position()) < ClimbConstants.POSITION_TOLERANCE.in(Meters);
  }

  /**
   * Gets the position of climb n meters
   *
   * @return Position of hardware in meters
   */
  @Logged
  public double position() {
    return hardware.getPosition();
  }

  /**
   * Updates the voltage usign pid and ff
   *
   * @param positionSetpoint The position to set the climb mechanism to
   */
  private void update(double positionSetpoint) {
    double goal =
        Double.isNaN(positionSetpoint) // checks if it is a number
            ? ClimbConstants.MIN_HEIGHT.in(Meters)
            : MathUtil.clamp(
                positionSetpoint,
                ClimbConstants.MIN_HEIGHT.in(Meters),
                ClimbConstants.MAX_HEIGHT.in(Meters));

    double pidSetpoint = controller.getSetpoint().velocity;
    double pidvolts = controller.calculate(hardware.getPosition(), goal);
    double ffVolts = ff.calculateWithVelocities(pidSetpoint, controller.getSetpoint().velocity);

    hardware.setVoltage(pidvolts + ffVolts);
  }

  /**
   * @return the setpoint of the PID
   */
  public double positionSetpoint() {
    return controller.getSetpoint().position;
  }

  /**
   * @param height The desired height
   * @return Move the climb
   */
  public Command goTo(DoubleSupplier height) {
    return run(() -> update(height.getAsDouble())).finallyDo(() -> hardware.setVoltage(0));
  }

  /**
   * A double instead of a double supplier
   *
   * @param height Height of the climb mechamism
   * @return Command to go to desired height
   */
  public Command goTo(double height) {
    return goTo(() -> height);
  }

  /**
   * Retracts climb to minium height
   *
   * @return Command to retract
   */
  public Command retractToMinHeight() {
    return goTo(ClimbConstants.MIN_HEIGHT.in(Meters)).withName("retracting");
  }

  /**
   * Extends climb to maxium height
   *
   * @return Command to extend
   */
  public Command extendToMaxHeight() {
    return goTo(ClimbConstants.MAX_HEIGHT.in(Meters)).withName("Extending");
  }

  @Override
  public void periodic() {
    climbVisualizer.setLength(positionSetpoint());
    measurement.setLength(position());

    if (TUNING) {
      ff.setKs(kS.get());
      ff.setKg(kG.get());
      ff.setKv(kV.get());
      ff.setKa(kA.get());
    }
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
