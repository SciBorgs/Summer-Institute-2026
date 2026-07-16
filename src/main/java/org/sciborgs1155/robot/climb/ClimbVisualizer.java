package org.sciborgs1155.robot.climb;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ClimbVisualizer {
  // put not logged
  @NotLogged private final Mechanism2d mech; // The physical field the robot is in
  private final String name;
  private final MechanismLigament2d climb; // The elevator mechanism

  public ClimbVisualizer(String name, Color8Bit color) {
    this.name = name;

    mech = new Mechanism2d(50, 50); // 50 by 50 field

    MechanismRoot2d root = mech.getRoot("chasis", 2, 0);
    climb =
        root.append(
            new MechanismLigament2d(
                "climb",
                ClimbConstants.MAX_HEIGHT.in(Meters) * 10, // multiply by 10 to scale
                90,
                3,
                color));
  }

  public void setLength(double length) {
    climb.setLength(length * 10);
    SmartDashboard.putData("Robot/climb/" + name, mech);
  }
}
