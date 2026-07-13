package org.sciborgs1155.robot.hood;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.hood.HoodConstants.*;

public class SimHood implements HoodIO{
    private final SingleJointedArmSim sim;

    public SimHood(){
        sim = 
        new SingleJointedArmSim(
            DCMotor.getKrakenX44(1),
            GEARING,
            MOI,
            HOOD_RADIUS.in(Meters),
            MIN_ANGLE.in(Radians),
            MAX_ANGLE.in(Radian),
            true,
            STARTING_ANGLE.in(Radians)
        );
    }

    @Override
    public double angle() {
        return sim.getAngleRads();
    }

    @Override
    public void setVoltage(double volts) {
        sim.setInputVoltage(volts);
        sim.update(PERIOD.in(Seconds));
    }

    @Override
    public double velocity() {
        return sim.getVelocityRadPerSec();
    }

    @Override
    public void close() throws Exception {}
    
}
