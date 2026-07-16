package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class ShooterConstants {

  public static final double GEAR_RATIO = 1 / .8;
  public static final Current STATOR_CURRENT_LIMIT = Amps.of(30);
  public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(30);
  public static final AngularVelocity MAX_VELOCITY = RPM.of(7230);
  public static final double MAX_VOLTAGE = 12.0; // from rebuilt
  public static final AngularVelocity VELOCITY_TOLERANCE = RadiansPerSecond.of(1);
  public static final AngularVelocity IDLE_VELOCITY = RadiansPerSecond.of(5);
  public static final double SENSOR_MECHANISM_RATIO = GEAR_RATIO * 2 * Math.PI; // in radians

  public static class VelocityControl {
    public static final double P = 0.03;
    public static final double I = 0.0;
    public static final double D = 0.0;

    public static final double S = 0.0;
    public static final double V = 0.016981;
    public static final double A = 0.0021296;
  }
}
