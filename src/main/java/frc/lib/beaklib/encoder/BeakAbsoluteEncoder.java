// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.encoder;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.beaklib.motor.DataSignal;
import frc.lib.beaklib.units.AngularVelocity;

/** Generic external absolute encoder. */
public interface BeakAbsoluteEncoder {
    /**
     * Get the encoder's position. Depending on the configuration,
     * this may be either relative (based on the magnet offset), or absolute.
     * 
     * @param latencyCompensated Whether or not to attempt to latency-compensate the encoder position.
     * 
     * @return Position.
     */
    public DataSignal<Rotation2d> getEncoderPosition(boolean latencyCompensated);

    /**
     * Set the encoder's position.
     * 
     * @param position
     *            New position.
     */
    public void setEncoderPosition(Rotation2d position);

    /**
     * Get the encoder's velocity.
     * 
     * @return Velocity of the absolute encoder.
     */
    public DataSignal<AngularVelocity> getEncoderVelocity();

    /**
     * Get the encoder's absolute position, independent of calls to setPosition().
     * 
     * @return Absolute position.
     */
    public DataSignal<Rotation2d> getAbsoluteEncoderPosition(boolean latencyCompensated);

    /**
     * Configure the zero point of the absolute position.
     * 
     * @param offset The zero point of the absolute position.
     */
    public void setAbsoluteOffset(Rotation2d offset);

    /**
     * Get the zero point of the absolute position.
     *
     * @return Zero point
     */
    public Rotation2d getAbsoluteOffset();

    /**
     * Set the period in which updated data is sent to the CAN bus or motor.
     * 
     * @param period
     *            Period in ms.
     */
    public void setDataFramePeriod(int period);

    /**
     * Set the direction in which positive rotation is reported.
     * 
     * @param cwPositive
     *            False means counter-clockwise rotation results in positive
     *            rotation,
     *            true means it results in negative rotation.
     */
    public void setCWPositive(boolean cwPositive);

    /**
     * Restore factory defaults of the encoder.
     */
    public void restoreFactoryDefault();
}
