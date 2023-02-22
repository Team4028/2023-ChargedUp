// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;

/** Standard WPILib Gyro with pitch & roll methods. */
public interface BeakGyro extends Gyro {
    /**
     * Return the pitch of the robot.
     * 
     * <p>Note that if your gyro is rotated, you may have to switch pitch and roll.
     * 
     * @return The current pitch of the robot.
     */
    public Rotation2d getPitchRotation2d();

    /**
     * Return the roll of the robot.
     * 
     * <p>Note that if your gyro is rotated, you may have to switch pitch and roll.
     * 
     * @return The current roll of the robot.
     */
    public Rotation2d getRollRotation2d();
}
