// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.encoder;

/** Generic external absolute encoder. */
public interface BeakAbsoluteEncoder {
    /**
     * Get the encoder's position. Depending on the configuration,
     * this may be either relative (based on the magnet offset), or absolute.
     * 
     * @return Position, in radians.
     */
    public double getPosition();

    /**
     * Set the encoder's position.
     * 
     * @param position New position, radians.
     */
    public void setEncoderPosition(double position);

    /**
     * Get the encoder's velocity.
     * 
     * @return Velocity, usually in degrees per second.
     */
    public double getVelocity();

    /**
     * Get the encoder's absolute position, independent of calls to setPosition().
     * 
     * @return Absolute position, in radians.
     */
    public double getAbsolutePosition();

    /**
     * Configure the zero point of the absolute position.
     * 
     * @param degrees
     */
    public void setAbsoluteOffset(double degrees);

    /**
     * Get the zero point of the absolute position.
     *
     * @return Zero point
     */
    public double getAbsoluteOffset();

    /**
     * Set the period in which updated data is sent to the CAN bus or motor.
     * 
     * @param period Period in ms.
     */
    public void setDataFramePeriod(int period);

    /**
     * Set the direction in which positive rotation is reported.
     * 
     * @param direction False means counter-clockwise rotation results in positive
     *                  rotation,
     *                  true means it results in negative rotation.
     */
    public void setSensorDirection(boolean direction);

    /**
     * Restore factory defaults of the encoder.
     */
    public void restoreFactoryDefault();
}
