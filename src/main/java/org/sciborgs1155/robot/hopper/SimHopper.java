package org.sciborgs1155.robot.hopper;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;



public class SimHopper implements HopperIO{
    private static final DCMotorSim motor;


    public  SimHopper {
        motor = new DCMotorSim(null, null, null);
    }
    @Override
    public void setVoltage(double volts){
        sim.setInputVoltage(volts);
        sim.update(PERIOD.in(Seconds));

    }

    @Override
    public double velocity(){
        return sim.getVelocityRadPerSec();
    }

    @Override
    public void close() throws Exception{

    }
}