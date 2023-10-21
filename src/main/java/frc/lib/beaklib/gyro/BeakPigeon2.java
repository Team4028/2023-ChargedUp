// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.gyro;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.beaklib.motor.DataSignal;
import frc.lib.beaklib.units.AngularVelocity;

/** A standard CTRE Pigeon 2 implemented as a BeakGyro. */
public class BeakPigeon2 extends WPI_Pigeon2 implements BeakGyro {
    public BeakPigeon2(int port, String canBus) {
        super(port, canBus);
    }

    public BeakPigeon2(int port) {
        super(port);
    }

    @Override
    public DataSignal<Rotation2d> getPitchRotation2d(boolean latencyCompensated) {
        return new DataSignal<Rotation2d>(Rotation2d.fromDegrees(getPitch()));
    }

    @Override
    public DataSignal<Rotation2d> getRollRotation2d(boolean latencyCompensated) {
        return new DataSignal<Rotation2d>(Rotation2d.fromDegrees(getRoll()));
    }

    @Override
    public DataSignal<Rotation2d> getYawRotation2d(boolean latencyCompensated) {
        return new DataSignal<Rotation2d>(getRotation2d());
    }

    @Override
    public DataSignal<AngularVelocity> getAngularVelocity() {
        return new DataSignal<AngularVelocity>(AngularVelocity.fromDegreesPerSecond(getRate()));
    }
}
