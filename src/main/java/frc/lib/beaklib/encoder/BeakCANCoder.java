// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.encoder;

import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.beaklib.motor.DataSignal;
import frc.lib.beaklib.units.AngularVelocity;

/** CANCoder, as a {@link BeakAbsoluteEncoder}. */
public class BeakCANCoder extends WPI_CANCoder implements BeakAbsoluteEncoder {
    public BeakCANCoder(int deviceNumber) {
        this(deviceNumber, "");
    }

    public BeakCANCoder(int deviceNumber, String CANBus) {
        super(deviceNumber, CANBus);
    }

    @Override
    public DataSignal<Rotation2d> getAbsoluteEncoderPosition(boolean latencyCompensated) {
        return new DataSignal<Rotation2d>(new Rotation2d(Math.toRadians(super.getAbsolutePosition())));
    }

    @Override
    public void setEncoderPosition(Rotation2d position) {
        super.setPosition(position.getDegrees());
    }

    @Override
    public void setAbsoluteOffset(Rotation2d offset) {
        super.configMagnetOffset(offset.getDegrees());
    }

    @Override
    public void setDataFramePeriod(int period) {
        super.setStatusFramePeriod(CANCoderStatusFrame.SensorData, period);
    }

    @Override
    public void setCWPositive(boolean direction) {
        super.configSensorDirection(direction);
    }

    @Override
    public void restoreFactoryDefault() {
        super.configFactoryDefault();
    }

    @Override
    public Rotation2d getAbsoluteOffset() {
        return Rotation2d.fromDegrees(super.configGetMagnetOffset());
    }

    @Override
    public DataSignal<Rotation2d> getEncoderPosition(boolean latencyCompensated) {
        return new DataSignal<Rotation2d>(new Rotation2d(Math.toRadians(super.getPosition())));
    }

    @Override
    public DataSignal<AngularVelocity> getEncoderVelocity() {
        AngularVelocity velocity = AngularVelocity.fromDegreesPerSecond(super.getVelocity());
        return new DataSignal<AngularVelocity>(velocity);
    }

}
