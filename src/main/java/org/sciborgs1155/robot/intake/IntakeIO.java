package org.sciborgs1155.robot.intake;

public interface IntakeIO extends AutoCloseable {
    void setVoltage(double voltage);

    double velocity();
}
