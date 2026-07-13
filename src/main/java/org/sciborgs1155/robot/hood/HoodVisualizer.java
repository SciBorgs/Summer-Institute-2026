package org.sciborgs1155.robot.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static org.sciborgs1155.robot.hood.HoodConstants.HOOD_RADIUS;
import static org.sciborgs1155.robot.hood.HoodConstants.STARTING_ANGLE;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** visualizes the hood in simulation */
public class HoodVisualizer {
    private String name;
    private Mechanism2d mechanism;
    private MechanismLigament2d hood;
    private MechanismLigament2d fuelTrajectory;

    /**
     * @param name of visualizer
     * @param hoodColor of hood
     * @param trajColor of line depicting fuel trajectory
     */
    public HoodVisualizer(String name, Color8Bit hoodColor, Color8Bit trajColor){
        this.name = name;
        mechanism = new Mechanism2d(100, 100);
        MechanismRoot2d chassis = mechanism.getRoot("Chassis", 50, 10);
        hood =
            chassis.append(
                new MechanismLigament2d("hood", 
                HOOD_RADIUS.in(Inches)*5, STARTING_ANGLE.in(Degrees), 3, hoodColor
                )
            );
        fuelTrajectory = 
            hood.append(
                new MechanismLigament2d("fuel trajectory", 25, 90, 2, trajColor)
            );
    }

    /**
     * @param angleDegrees angle to set visualizer to (degrees (duh))
     */
    public void setAngle(double angleDegrees){
        hood.setAngle(angleDegrees);
        SmartDashboard.putData("/Robot/hood/" + name, mechanism);
    }
}

