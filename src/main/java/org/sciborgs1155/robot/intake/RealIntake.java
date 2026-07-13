package org.sciborgs1155.robot.intake;

import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.intake.IntakeConstants.*;
import static org.sciborgs1155.robot.Ports.Intake.*;

public class RealIntake implements IntakeIO {
    private final TalonFX Roller;
    public RealIntake() {
        Roller = new TalonFX(ROLLER);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // needs testing depending on robot's intake design.
        config.Feedback.SensorToMechanismRatio = GEARING;
        Roller.getConfigurator().apply(config);

        FaultLogger.register(Roller);
        TalonUtils.addMotor(Roller);
    }

    @Override
    public void setVoltage(double voltage) {
        Roller.setVoltage(voltage);
    }

    @Override
    public double velocity() {
        return Roller.getVelocity().getValueAsDouble();
    }

    @Override
    public void close() throws Exception {
        Roller.close();
    }
}
