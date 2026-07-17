package org.sciborgs1155.robot.hopper;
import org.sciborgs1155.lib.SimpleMotor;
import org.sciborgs1155.lib.Beambreak;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import static org.sciborgs1155.robot.Ports.Hopper.*;
import static org.sciborgs1155.robot.hopper.HopperConstants.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.sciborgs1155.robot.Robot;

//Code
@Logged
public final class Hopper extends SubsystemBase implements AutoClosable{
    public final TalonFX motor;
    private final SimpleMotor hardware;
    private final Beambreak beambreak;
    public final Trigger blocked;

    //Hopper
    public Hopper() {
        motor = new TalonFX();
        TalonFXConfiguration configs = new TalonFXConfiguration();
        SimpleMotor HopperMotor = new talon(motor, );
    }
    

}