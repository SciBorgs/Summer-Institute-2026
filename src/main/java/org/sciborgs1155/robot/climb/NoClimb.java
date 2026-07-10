package org.sciborgs1155.robot.climb;

public class NoClimb implements ClimbIO {

    @Override
    public void setVoltage(double voltage) {}

    @Override
    public double getPosition() {
        return 0;
    }

    @Override
    public double velocity() {
        return 0;
    }
    
    @Override
    public void close() throws Exception {}

    
}
