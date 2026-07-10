package org.sciborgs1155.robot.climb;


import static edu.wpi.first.units.Units.Amps;

import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class RealClimb implements ClimbIO{

    private final TalonFX rightMotor;
    private final TalonFX leftMotor; 

    /* Configs & Initialization */
    public RealClimb() {
        rightMotor = new TalonFX(33); //for now 
        leftMotor = new TalonFX(34); //for now

        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // stops the motor
        configs.MotorOutput.Inverted =
            InvertedValue.CounterClockwise_Positive; // counterclockwise is postive
        configs.CurrentLimits.StatorCurrentLimit = ClimbConstants.STATOR_LIMIT.in(Amps);
        configs.CurrentLimits.SupplyCurrentLimit = ClimbConstants.SUPPLY_LIMIT.in(Amps);
        configs.Feedback.SensorToMechanismRatio = ClimbConstants.SENSOR_TO_MECHANISM_RATIO; 

        rightMotor.setControl(new Follower(34, MotorAlignmentValue.Aligned));

        rightMotor.getConfigurator().apply(configs);
        leftMotor.getConfigurator().apply(configs);

        FaultLogger.register(rightMotor);
        FaultLogger.register(leftMotor);

        TalonUtils.addMotor(rightMotor);
        TalonUtils.addMotor(leftMotor);
    }

    @Override
    public void setVoltage(double voltage) {
        rightMotor.setVoltage(voltage);

    }

    @Override
    public double getPosition() {
        return rightMotor.getPosition().getValueAsDouble() * ClimbConstants.SENSOR_TO_MECHANISM_RATIO;
    }

    @Override
    public double velocity() {
        return leftMotor.getVelocity().getValueAsDouble() * ClimbConstants.SENSOR_TO_MECHANISM_RATIO;
    }

    @Override
    public void close() throws Exception {
        rightMotor.close();
    }


}
    
