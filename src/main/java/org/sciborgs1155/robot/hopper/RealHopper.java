package org.sciborgs1155.robot.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;
import org.sciborgs1155.robot.hopper.HopperConstants;
import static edu.wpi.first.units.Units.Amps;


// subpackages

//external imports/packages
import static edu.wpi.first.units.Units.RadiansPerSecond;

//code

public class RealHopper implements HopperIO{
    private final TalonFX motor;
   
    
    public RealHopper(){
        motor = new TalonFX(35);
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = HopperConstants.SUPPLY_LIMIT.in(Amps);
        config.CurrentLimits.StatorCurrentLimit = HopperConstants.STATOR_LIMIT.in(Amps);
        config.Feedback.SensorToMechanismRatio  = HopperConstants.GEARING;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        motor.getConfigurator().apply(config);
        FaultLogger.register(motor);
        TalonUtils.addMotor(motor);
    }


    @Override
    public void setVoltage(double voltage){
        motor.setVoltage(voltage);
    }
    @Override
    public double velocity(){
        return motor.getVelocity().getValue().in(RadiansPerSecond);
    }
    @Override
    public void close() throws Exception{
        motor.close();
    }
}