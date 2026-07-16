package org.sciborgs1155.robot.climb;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class SimClimb implements ClimbIO {
  private final ElevatorSim clibmSim;

  public SimClimb() {
    clibmSim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                DCMotor.getKrakenX60(2),
                ClimbConstants.WEIGHT.in(Kilograms),
                ClimbConstants.SPROCKET_RADIUS.in(Meters),
                ClimbConstants.GEARING),
            DCMotor.getKrakenX60(2),
            ClimbConstants.MIN_HEIGHT.in(Meters),
            ClimbConstants.MAX_HEIGHT.in(Meters),
            false,
            ClimbConstants.STARTING_HEIGHT.in(Meters));
  }

  @Override
  public void setVoltage(double voltage) {
    clibmSim.setInputVoltage(voltage);
    clibmSim.update(PERIOD.in(Seconds));
  }

  @Override
  public double getPosition() {
    return clibmSim.getPositionMeters();
  }

  @Override
  public double velocity() {
    return clibmSim.getVelocityMetersPerSecond();
  }

  @Override
  public void close() throws Exception {}
}
