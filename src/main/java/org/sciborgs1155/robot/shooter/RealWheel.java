package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.Ports.Shooter.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

public class RealWheel implements WheelIO {

  private final TalonFX flywheelMotor1;
  private final TalonFX flywheelMotor2;

  // motor configurations
  public RealWheel() {

    // change later
    flywheelMotor1 = new TalonFX(WHEEL_MOTOR1);
    flywheelMotor2 = new TalonFX(WHEEL_MOTOR2);

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // stops the motor
    configs.MotorOutput.Inverted =
        InvertedValue.CounterClockwise_Positive; // counterclockwise is postive
    configs.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT.in(Amps);
    configs.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT.in(Amps);

    flywheelMotor2.setControl(new Follower(30, MotorAlignmentValue.Aligned)); // links the motors

    /*
    TO DO: More configurations
     */

    flywheelMotor1.getConfigurator().apply(configs);

    /* Checks the motors */
    FaultLogger.register(flywheelMotor1);

    /* adds motors to a list of all global motors */
    TalonUtils.addMotor(flywheelMotor1);
  }

  @Override
  public void setVoltage(double voltage) {
    flywheelMotor1.setVoltage(voltage);
  }

  @Override
  public double Velocity() {
    return flywheelMotor1.getVelocity().getValueAsDouble();
  }

  @Override
  public void close() throws Exception {
    flywheelMotor1.close();
    flywheelMotor2.close();
  }
}
