package org.sciborgs1155.robot.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.sciborgs1155.robot.Robot;

import static org.sciborgs1155.robot.intake.IntakeConstants.*;

public class Intake extends SubsystemBase implements AutoCloseable  {
    private final IntakeIO hardware;

    public Intake(IntakeIO hardware) {
        this.hardware = hardware;
    }

    public static Intake create() {
        return new Intake(Robot.isReal() ? new RealIntake() : new NoIntake()) ;
    }

    public Command spin(double volts) {
        return run(() -> hardware.setVoltage(volts)).withName("spinning");
    }

    public Command stop() {
        return spin(0);
    }

    public Command intake() {
        return spin(INTAKE_POWER);
    }

    public Command outtake() {
        return spin(-INTAKE_POWER);
    }

    public static Intake none() {
        return new Intake(new NoIntake());
    }

    @Override
    public void close() throws Exception {
        hardware.close();
    }
}
