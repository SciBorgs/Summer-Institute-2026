package org.sciborgs1155.robot.climb;

public interface ClimbIO extends AutoCloseable {

    /**
     * sets voltage of the motor
     * 
     * @param voltage The voltage
     */
    void setVoltage(double voltage); 

    /**
     * 
     * @return The position in meters
     */
    double getPosition();

    /**
     * 
     * @return The velocity in m/s
     */
    double velocity();

    
}
