package org.sciborgs1155.robot.hood;

public interface HoodIO extends AutoCloseable{
    /**
     * @return angle of hoods (rads)
     */
    double angle();

    /**
     * !@param voltage set voltage of good motor
     */
    void setVoltage(double v);

    /**
     * @return current velocity in rads/sec
     */
    double velocity();

    @Override
    void close() throws Exception;
}
