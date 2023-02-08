// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.encoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

/** CANCoder, as a {@link BeakAbsoluteEncoder}. */
public class BeakAnalogInput extends AnalogInput implements BeakAbsoluteEncoder {
    private double offset;

    public BeakAnalogInput(int deviceNumber) {
        super(deviceNumber);
    }

    @Override
    public void setEncoderPosition(double position) {}

    @Override
    public void setAbsoluteOffset(double degrees) {
        System.out.println("Offset: " + degrees);
        offset = Units.degreesToRadians(degrees);
    }

    @Override
    public void setDataFramePeriod(int period) {}

    @Override
    public void setSensorDirection(boolean direction) {}

    @Override
    public void restoreFactoryDefault() {}

    @Override
    public double getPosition() {
        // System.out.println("abs Position: " + getAbsolutePosition());
        return getAbsolutePosition() + offset;
    }

    @Override
    public double getVelocity() {
        return 0.;
    }

    @Override
    public double getAbsolutePosition() {
        // SUSSY
        return (1.0 - super.getVoltage() / RobotController.getVoltage5V()) * 2.0 * Math.PI;
    }

    @Override
    public double getAbsoluteOffset() {
        return offset;
    }

}
