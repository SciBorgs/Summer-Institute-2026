package org.sciborgs1155.robot.shooter;

public interface WheelIO extends AutoCloseable {
  /**
   * Set voltage
   *
   * @param voltage The Voltage
   */
  void setVoltage(double voltage);

  /**
   * Gets flywheel velocity in radians per second
   *
   * @returns Velocity in radians per second
   */
  double Velocity();
}
