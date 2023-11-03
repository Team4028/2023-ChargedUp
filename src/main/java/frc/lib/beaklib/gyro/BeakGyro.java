// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.lib.beaklib.motor.DataSignal;

/** Standard WPILib Gyro with pitch & roll methods. */
public interface BeakGyro extends Gyro {
    /**
     * Return the pitch of the robot.
     * 
     * <p>
     * Note that if your gyro is rotated, you may have to switch pitch and roll.
     * 
     * @param latencyCompensated Whether or not to latency-compensate the gyro reading.
     * 
     * @return The current pitch of the robot.
     */
    public DataSignal<Rotation2d> getPitchRotation2d(boolean latencyCompensated);

    /**
     * Return the roll of the robot.
     * 
     * <p>
     * Note that if your gyro is rotated, you may have to switch pitch and roll.
     * 
     * @param latencyCompensated Whether or not to latency-compensate the gyro reading.
     * 
     * @return The current roll of the robot.
     */
    public DataSignal<Rotation2d> getRollRotation2d(boolean latencyCompensated);

    /**
     * Return the yaw of the robot.
     * 
     * <p>
     * Note that depending on your gyro's orientation/implementation, you may have to invert this.
     * Call <code>setGyroInverted</code> to invert the gyro yaw.
     * 
     * @param latencyCompensated Whether or not to latency-compensate the gyro reading.
     * 
     * @return The current yaw of the robot.
     */
    public DataSignal<Rotation2d> getYawRotation2d(boolean latencyCompensated);
    
    /**
     * Get the angular velocity of the robot.
     * 
     * @return The current angular velocity of the robot.
     */
    public DataSignal<Measure<Velocity<Angle>>> getAngularVelocity();
}
