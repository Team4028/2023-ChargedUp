// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.gyro;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.beaklib.motor.DataSignal;
import frc.lib.beaklib.units.AngularVelocity;

/** A v6 CTRE Pigeon 2 implemented as a BeakGyro. */
public class BeakV6Pigeon2 extends Pigeon2 implements BeakGyro {
    public BeakV6Pigeon2(int port) {
        super(port);
    }

    public BeakV6Pigeon2(int port, String canBus) {
        super(port, canBus);
    }

    @Override
    public DataSignal<Rotation2d> getPitchRotation2d(boolean latencyCompensated) {
        double pitchValue;
        double pitchTime;
        StatusSignal<Double> pitch = getPitch();

        if (latencyCompensated) {
            // this isn't documented well, but:
            // pitch is about Y
            // roll is about X
            // yaw is about Z
            pitchValue = StatusSignal.getLatencyCompensatedValue(pitch, getAngularVelocityY());
            pitchTime = RobotController.getFPGATime() / 1000000.;
        } else {
            pitchValue = pitch.getValue();
            pitchTime = pitch.getTimestamp().getTime();
        }

        Rotation2d rotation = Rotation2d.fromDegrees(pitchValue);
        return new DataSignal<Rotation2d>(rotation, pitchTime);
    }

    @Override
    public DataSignal<Rotation2d> getRollRotation2d(boolean latencyCompensated) {
        double rollValue;
        double rollTime;
        StatusSignal<Double> roll = getRoll();

        if (latencyCompensated) {
            rollValue = StatusSignal.getLatencyCompensatedValue(roll, getAngularVelocityX());
            rollTime = RobotController.getFPGATime() / 1000000.;
        } else {
            rollValue = roll.getValue();
            rollTime = roll.getTimestamp().getTime();
        }

        Rotation2d rotation = Rotation2d.fromDegrees(rollValue);
        return new DataSignal<Rotation2d>(rotation, rollTime);
    }

    @Override
    public DataSignal<Rotation2d> getYawRotation2d(boolean latencyCompensated) {
        double yawValue;
        double yawTime;
        StatusSignal<Double> yaw = getYaw();

        if (latencyCompensated) {
            yawValue = StatusSignal.getLatencyCompensatedValue(yaw, getAngularVelocityZ());
            yawTime = RobotController.getFPGATime() / 1000000.;
        } else {
            yawValue = yaw.getValue();
            yawTime = yaw.getTimestamp().getTime();
        }

        Rotation2d rotation = Rotation2d.fromDegrees(yawValue);
        return new DataSignal<Rotation2d>(rotation, yawTime);
    }

    @Override
    public DataSignal<AngularVelocity> getAngularVelocity() {
        StatusSignal<Double> angularVelocity = super.getAngularVelocityZ();

        return new DataSignal<AngularVelocity>(AngularVelocity.fromDegreesPerSecond(angularVelocity.getValue()), angularVelocity.getTimestamp().getTime());
    }

}
