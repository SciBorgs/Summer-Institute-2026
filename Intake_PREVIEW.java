// ============================================================================
// PREVIEW ONLY — this file is NOT part of the build.
//
// It lives at the project root (outside src/main/java), so Gradle ignores it.
// It shows a fully complete roller Intake system, written to drop straight
// into this codebase. It contains TWO files' worth of code:
//
//   1. Intake.java          -> replaces src/main/java/org/sciborgs1155/robot/intake/Intake.java
//   2. IntakeConstants.java -> NEW file in the same intake folder (your current
//                              Intake.java already references it, so it must
//                              exist for the project to compile)
//
// Everything here matches the existing conventions:
//   - Ports.Intake.ROLLER (already defined as 21 in Ports.java)
//   - TalonFX + TalonFXConfiguration, like RealWheel.java
//   - FaultLogger.register() + TalonUtils.addMotor(), like TalonModule/RealWheel
//   - Instance command factories with .withName(), like Shooter.java
//   - @Logged getters for Epilogue, like Shooter.java
//   - A systemsCheck() Test using Assertion.tAssert(), like Shooter.goToTest()
//
// To wire it into Robot.java (not done here):
//   private final Intake intake = new Intake();
//   operator.rightTrigger().whileTrue(intake.intake());
//   operator.leftTrigger().whileTrue(intake.outtake());
// ============================================================================


// ╔══════════════════════════════════════════════════════════════════════════╗
// ║ FILE 1: src/main/java/org/sciborgs1155/robot/intake/Intake.java          ║
// ╚══════════════════════════════════════════════════════════════════════════╝

package org.sciborgs1155.robot.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.sciborgs1155.lib.Assertion.tAssert;
import static org.sciborgs1155.robot.intake.IntakeConstants.CURRENT_LIMIT;
import static org.sciborgs1155.robot.intake.IntakeConstants.INTAKE_VOLTAGE;
import static org.sciborgs1155.robot.intake.IntakeConstants.MINIMUM_TEST_SPEED;
import static org.sciborgs1155.robot.intake.IntakeConstants.OUTTAKE_VOLTAGE;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Set;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.lib.Assertion.TruthAssertion;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;
import org.sciborgs1155.lib.Test;
import org.sciborgs1155.robot.Ports;

/** A single-motor roller intake driven by a TalonFX. */
public class Intake extends SubsystemBase implements AutoCloseable {

  private final TalonFX roller;

  public Intake() {
    roller = new TalonFX(Ports.Intake.ROLLER);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    roller.getConfigurator().apply(config);

    /* Checks the motor */
    FaultLogger.register(roller);

    /* adds motor to a list of all global motors */
    TalonUtils.addMotor(roller);

    // When no other command needs the intake, keep the rollers stopped.
    setDefaultCommand(stop().withName("idle"));
  }

  /**
   * Spins the roller at a given voltage.
   *
   * @param volts The voltage to apply to the roller motor.
   * @return A command that spins the roller until interrupted.
   */
  public Command spin(double volts) {
    return run(() -> roller.setVoltage(volts)).withName("spinning");
  }

  /**
   * Spins the roller inward to pick up game pieces.
   *
   * @return A command that runs the intake until interrupted.
   */
  public Command intake() {
    return spin(INTAKE_VOLTAGE).withName("intaking");
  }

  /**
   * Spins the roller outward to eject game pieces.
   *
   * @return A command that runs the intake in reverse until interrupted.
   */
  public Command outtake() {
    return spin(OUTTAKE_VOLTAGE).withName("outtaking");
  }

  /**
   * Stops the roller.
   *
   * @return A command that holds the roller stopped.
   */
  public Command stop() {
    return run(() -> roller.setVoltage(0)).withName("stopped");
  }

  /**
   * @return The roller velocity (in rotations per second).
   */
  @Logged
  public double velocity() {
    return roller.getVelocity().getValueAsDouble();
  }

  /**
   * @return The supply current drawn by the roller motor (in amps).
   */
  @Logged
  public double current() {
    return roller.getSupplyCurrent().getValueAsDouble();
  }

  /**
   * Systems check: runs the intake and asserts the roller actually spins up.
   *
   * @return Test for use in a systems check.
   */
  public Test systemsCheck() {
    Command testCommand = intake().withTimeout(2);
    TruthAssertion spinning =
        tAssert(
            () -> velocity() > MINIMUM_TEST_SPEED.in(RotationsPerSecond),
            "Intake Syst Check (roller speed)",
            () ->
                "expected: > "
                    + MINIMUM_TEST_SPEED.in(RotationsPerSecond)
                    + " rps; actual: "
                    + velocity());
    return new Test(testCommand, Set.of(spinning));
  }

  /** closes motor */
  @Override
  public void close() throws Exception {
    roller.close();
  }
}


// ╔══════════════════════════════════════════════════════════════════════════╗
// ║ FILE 2: src/main/java/org/sciborgs1155/robot/intake/IntakeConstants.java ║
// ║ (NEW file — your current Intake.java already references CURRENT_LIMIT)   ║
// ╚══════════════════════════════════════════════════════════════════════════╝

package org.sciborgs1155.robot.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class IntakeConstants {

  /** Supply current limit for the roller motor. */
  public static final Current CURRENT_LIMIT = Amps.of(40);

  /** Voltage applied while intaking. */
  public static final double INTAKE_VOLTAGE = 8.0;

  /** Voltage applied while ejecting (negative = reverse). */
  public static final double OUTTAKE_VOLTAGE = -6.0;

  /** Minimum roller speed for the systems check to pass. */
  public static final AngularVelocity MINIMUM_TEST_SPEED = RotationsPerSecond.of(5);
}
