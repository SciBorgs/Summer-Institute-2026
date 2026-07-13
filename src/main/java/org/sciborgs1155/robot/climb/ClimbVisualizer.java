package org.sciborgs1155.robot.climb;

import static edu.wpi.first.units.Units.Meters;

import com.reduxrobotics.sensors.canandcolor.ColorFrame;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ClimbVisualizer {
    //put not logged
    private final Mechanism2d mech; // The physical field the robot is in
    private final String name;
    private final MechanismLigament2d climb; // The elevator mechanism

    

    public ClimbVisualizer(String name, Color8Bit color) {
        this.name = name;

        mech = new Mechanism2d(50, 50);
        climb = new MechanismLigament2d("climb", ClimbConstants.MAX_HEIGHT.in(Meters) * 10, 90, 3, color); //multiply by 10 to scale

    }

}
