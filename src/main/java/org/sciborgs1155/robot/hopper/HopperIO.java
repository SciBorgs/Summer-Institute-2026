package org.sciborgs1155.robot.hopper;

public interface HopperIO extends AutoCloseable{
    /*
    *
    * Sets Voltage
    * @param voltage Voltage
    */
    void setVoltage(double voltage);
    /*
    * gets the curretn velocity in radians per sec 
    * @return velocity in radins per sec
    * 
    */
    double velocity();

    @Override
    void close() throws Exception;
}