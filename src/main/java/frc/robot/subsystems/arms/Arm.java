// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arms;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The upper Argos Arm
 */
public abstract class Arm extends SubsystemBase {

    public static final double EXTEND_COEFFICIENT = 116.1;
    public static final double RETRACT_COEFFICIENT = 156.58;
    public static final double EXTEND_WAIT_INTERVAL = 0.2;
    public static final double RETRACT_WAIT_INTERVAL = 0.4;
    protected static final double OPEN_LOOP_RAMP_RATE = 0.1;
    protected static final double RAMP_RATE = 0.25;

    protected SparkMaxPIDController m_pid;

    protected CANSparkMax m_motor;
    protected RelativeEncoder m_encoder;
    protected double m_targetPosition, m_distanceToTravel = 0;
    public ElevatorFeedforward FFModel;

    /**
     * the enum containing the desired positions of the arm
     */

    /**
     * Runs the initial setup that would ordinarily
     * take place in the constructor of a traditional
     * subsystem class
     * <p>
     * MUST be run after initializing the SparkMAX
     */
    protected void initArm() {
        m_encoder = m_motor.getEncoder();

        m_motor.setSmartCurrentLimit(40);
        m_motor.setIdleMode(IdleMode.kBrake);

        m_motor.setOpenLoopRampRate(OPEN_LOOP_RAMP_RATE);
        m_motor.setClosedLoopRampRate(RAMP_RATE);

        m_motor.burnFlash();
        m_targetPosition = 3.0;
    }

    /**
     * runs the arm with raw vbus
     * 
     * @param speed
     *            the speed at which to run the arm
     */
    public void runArmVbus(double speed) {
        m_motor.set(speed);
    }

    /**
     * 
     * @return the motor current of the arm motor in amps
     */
    public double getMotorCurrent() {
        return m_motor.getOutputCurrent();
    }

    /**
     * Runs the motor to a place with position PID
     * 
     * @param position
     *            the encoder position to run it to
     */
    public void runToPosition(double position) {
        m_pid.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    /**
     * Runs the motor to a place with position PID & feedForward
     * 
     * @param position
     *            the encoder position to run it to
     * @param feedForward
     *            The feedforward value, in volts.
     */
    public void runToPosition(double position, double feedForward) {
        m_pid.setReference(position, CANSparkMax.ControlType.kPosition, 0, feedForward);
    }

    public boolean atTargetPosition() {
        return getError() < 0.2;
    }

    public BooleanSupplier atTargetPositionSupplier() {
        return (() -> atTargetPosition());
    }

    public BooleanSupplier isReady() {
        return () -> m_distanceToTravel > 1.0 ? getError() < 0.75 * m_distanceToTravel : getError() <= 1.0;
    }

    public double getError() {
        return Math.abs(this.getEncoderPosition() - m_targetPosition);
    }

    /**
     * zeros the encoder
     */
    public void zeroEncoder() {
        m_encoder.setPosition(0.0);
    }

    /**
     * 
     * @return the position of the arm's encoder
     */
    public double getEncoderPosition() {
        return m_encoder.getPosition();
    }

    /**
     * Set the position of the encoder.
     * 
     * @param position
     *            The desired position, in rotations.
     */
    public void setEncoderPosition(double position) {
        m_encoder.setPosition(position);
    }

    /**
     * Get the goal position of the arm.
     * 
     * @return The goal position of the arm, in rotations.
     */
    public double getTargetPosition() {
        return m_targetPosition;
    }

    /**
     * Set the target commanded position of the arm.
     * 
     * @param rotations
     *            The target position, in rotations.
     */
    public void setTargetPosition(double rotations) {
        m_targetPosition = rotations;
    }

    /**
     * 
     * @return the double value of the distance the arm needs to travel
     */
    public double getDistanceToTravel() {
        return m_distanceToTravel;
    }

    /**
     * sets the distance to travel
     * 
     * @param dist
     *            the distance to travel
     */
    public void setDistanceToTravel(double dist) {
        m_distanceToTravel = dist;
    }

    public static Arm getInstance() {
        return null;
    }

    public double getZeroVbus() {
        return 0.0;
    }

    public double getZeroCurrentThreshold() {
        return 0.0;
    }

    /**
     * @return A Command to hold the arm at its current position. Used after running
     *         open loop to stay put and not drop with gravity.
     */
    public Command holdArmPosition() {
        return runOnce(() -> {
            m_motor.stopMotor();
            runToPosition(m_encoder.getPosition());
        });
    }

    /**
     * Change the current arm position.
     * 
     * @param delta
     *            The amount to change the position.
     * @return A {@link Command} that changes the current position by the delta.
     */
    public Command changePositionCommand(double delta) {
        return runOnce(() -> {
            setTargetPosition(m_targetPosition + delta);
            runToPosition(m_targetPosition);
        });
    }

    @Override
    public void periodic() {
    }
}
