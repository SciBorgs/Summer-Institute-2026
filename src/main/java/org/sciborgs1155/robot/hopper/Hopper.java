package org.sciborgs1155.robot.hopper;
import org.sciborgs1155.lib.SimpleMotor;
import org.sciborgs1155.lib.Beambreak;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import static org.sciborgs1155.robot.hopper.HopperConstants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.units.Units.Amps;
import edu.wpi.first.units.measure.Current;

import org.sciborgs1155.robot.Robot;

//Code
@Logged
public final class Hopper extends SubsystemBase implements AutoClosable{
    public final TalonFX motor;
    private final SimpleMotor hardware;
    

    //Hopper
    public Hopper(int deviceID) {
        motor = new TalonFX(deviceID);
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        configs.CurrentLimits.StatorCurrentLimit = HopperConstants.CURRENT_LIMIT.in(Amps);
        configs.CurrentLimits.StatorCurrentLimitEnable = true;
        this.hardware = SimpleMotor.talon(motor,configs);
    }
    public static Hopper none(){
        hardware = SimpleMotor.none();
    }
    public void set(){
        hardware.set(HopperConstants.INTAKING_POWER);
    }
    
    
    public Command runHopper(DoubleSupplier velocity, boolean IorO){
        return run(() -> {
            double v = velocity.getAsDouble();
            update(IorO ? v : -v);
        });
    }

    public Command stopHopper(){
        return run(() -> hardware.setVoltage(0));
    }
    
    public void close() {
        hardware.close();
    }

}



/*
1.You need configs -> you just declared it, but didn’t actually write the configurations    |This should be done
2. You need a run command                                                                   | I think this is done
3. You don’t need a setVoltage method                                                       | Done?
4. You need a stop Command (just set hardware to 0)                                         | Done.
5.  You need a create() method if robot is real
6. Outtake method (just opposite of run)                                                    | I think this is done
7. Javadocs                                                                                | | I dislike this but fine
*/