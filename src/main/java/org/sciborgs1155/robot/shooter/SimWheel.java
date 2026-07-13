package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.sciborgs1155.robot.shooter.ShooterConstants.VelocityControl;

public class SimWheel implements WheelIO {

  private final FlywheelSim simWheel;

  // Intialize
  public SimWheel() {
    simWheel =
        new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(
                VelocityControl.V, VelocityControl.A), // physical behavior
            DCMotor.getKrakenX60(1)); // eletrical behavior
  }

  @Override
  public void setVoltage(double voltage) {
    simWheel.setInputVoltage(voltage);
    simWheel.update(PERIOD.in(Seconds));
  }

  @Override
  public double Velocity() {
    return simWheel.getAngularVelocityRadPerSec();
  }

  @Override
  public void close() throws Exception {}
}
