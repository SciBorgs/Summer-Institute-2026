package org.sciborgs1155.robot.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.hood.HoodConstants.*;

import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** hood class w/ motor controller */
public class RealHood implements HoodIO{
    
    private final TalonFX motor;
    private final TalonFXConfiguration configs;

    public RealHood(){
        motor = new TalonFX(PORT);

        configs = new TalonFXConfiguration();

        configs.CurrentLimits.StatorCurrentLimit = STATOR_LIMIT.in(Amps);
        configs.CurrentLimits.SupplyCurrentLimit = SUPPLY_LIMIT.in(Amps);
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.Feedback.SensorToMechanismRatio = GEARING;
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        motor.getConfigurator().apply(configs);
        TalonUtils.addMotor(motor);
        FaultLogger.register(motor);
    }

    /** 
     * @return hood angle in rads
     */
    @Override
    public double angle() {
        return motor.getPosition().getValue().in(Radians);
    }

    /** 
     * @param v set motor voltage to given voltage
     */
    @Override
    public void setVoltage(double v) {
        motor.setVoltage(v);
    }

    /**
     * @return rotational velocity of hood (rad/sec)
     */
    @Override
    public double velocity() {
        return motor.getVelocity().getValue().in(RadiansPerSecond);
    }

    @Override
    public void close() throws Exception {
        motor.close();
    }

}
