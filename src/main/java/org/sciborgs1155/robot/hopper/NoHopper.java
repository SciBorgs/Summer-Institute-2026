package org.sciborgs1155.robot.hopper;

public class NoHopper implements HopperIO{
    @Override
    public void setVoltage(double voltage){
    
    }
    @Override
    public double velocity(){
        return 0;
    }
    @Override
    public void close() throws Exception{}
}