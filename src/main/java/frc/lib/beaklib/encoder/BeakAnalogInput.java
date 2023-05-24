// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.encoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.beaklib.motor.DataSignal;
import frc.lib.beaklib.units.AngularVelocity;

/** CANCoder, as a {@link BeakAbsoluteEncoder}. */
public class BeakAnalogInput extends AnalogInput implements BeakAbsoluteEncoder {
    private Rotation2d m_offset;

    public BeakAnalogInput(int deviceNumber) {
        super(deviceNumber);
    }

    @Override
    public DataSignal<Rotation2d> getAbsoluteEncoderPosition() {
        // SUSSY
        double radians = (1.0 - super.getVoltage() / RobotController.getVoltage5V()) * 2.0 * Math.PI + m_offset.getRadians();
        return new DataSignal<Rotation2d>(new Rotation2d(radians));
    }

    @Override
    public Rotation2d getAbsoluteOffset() {
        return m_offset;
    }

    @Override
    public DataSignal<Rotation2d> getEncoderPosition() {
        return getAbsoluteEncoderPosition();
    }

    @Override
    public void setEncoderPosition(Rotation2d position) {
    }

    @Override
    public DataSignal<AngularVelocity> getEncoderVelocity() {
        return new DataSignal<AngularVelocity>(new AngularVelocity());
    }

    @Override
    public void setAbsoluteOffset(Rotation2d offset) {
        m_offset = offset;
    }

    @Override
    public void setDataFramePeriod(int period) {
    }

    @Override
    public void setCWPositive(boolean cwPositive) {
    }

    @Override
    public void restoreFactoryDefault() {
    }

}
