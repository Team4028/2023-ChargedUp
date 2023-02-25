// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.gyro;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

/** A standard CTRE Pigeon 2 implemented as a BeakGyro. */
public class BeakPigeon2 extends WPI_Pigeon2 implements BeakGyro {
    public BeakPigeon2(int port, String canBus) {
        super(port, canBus);
    }

    public BeakPigeon2(int port) {
        super(port);
    }

    @Override
    public Rotation2d getPitchRotation2d() {
        return Rotation2d.fromDegrees(getPitch());
    }

    @Override
    public Rotation2d getRollRotation2d() {
        return Rotation2d.fromDegrees(getRoll());
    }
}
