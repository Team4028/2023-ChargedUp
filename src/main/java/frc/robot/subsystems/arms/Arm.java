// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arms;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The upper Argos Arm
 */
public abstract class Arm extends SubsystemBase {

    public static final Map<String, double[]> m_updatedArmPosMap = new HashMap<String, double[]>();

    public static final double EXTEND_COEFFICIENT = 116.1;
    public static final double RETRACT_COEFFICIENT = 156.58;
    public static final double EXTEND_WAIT_INTERVAL = 0.2;
    public static final double RETRACT_WAIT_INTERVAL = 0.4;
    protected static final double OPEN_LOOP_RAMP_RATE = 0.5;
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

        m_updatedArmPosMap.put("STOWED", new double[] { 3.0, 4.0, 305.0 });
        m_updatedArmPosMap.put("INTERMEDIATE_LOW", new double[] { 9.5, 21.5, 275.0 });
        m_updatedArmPosMap.put("SCORE_LOW_CUBE", new double[] { 15.0, 13.0, 200.0 });
        m_updatedArmPosMap.put("SCORE_MID_CUBE", new double[] { 39.0, 6.0, 215.0 });
        m_updatedArmPosMap.put("SCORE_HIGH_CUBE", new double[] { 51.0, 34.0, 203.0 });
        m_updatedArmPosMap.put("AUTON_PREP_CUBE", new double[] { 51.0, 4.0, 203.0 });
        m_updatedArmPosMap.put("ACQUIRE_FLOOR_CUBE", new double[] { 9.0, 23.0, 245.0 });
        m_updatedArmPosMap.put("ACQUIRE_FLOOR_CONE_TIPPED", new double[] { 8.0, 26.5, 260.0 });
        m_updatedArmPosMap.put("ACQUIRE_FLOOR_CONE_UPRIGHT", new double[] { 9.0, 19.6, 262.5 });
        m_updatedArmPosMap.put("AUTON_UPRIGHT_CONE", new double[] { 8.5, 19.6, 261.5 });
        m_updatedArmPosMap.put("SCORE_LOW_CONE", new double[] { 15.0, 13.0, 200.0 });
        m_updatedArmPosMap.put("SCORE_MID_CONE", new double[] { 39.0, 6.0, 215.0 });
        m_updatedArmPosMap.put("SCORE_HIGH_CONE", new double[] { 51.0, 34.0, 203.0 });
        m_updatedArmPosMap.put("AUTON_PREP_CONE", new double[] { 51.0, 4.0, 203.0 });
        m_updatedArmPosMap.put("ACQUIRE_SINGLE_SUBSTATION", new double[] { 3.6, 2.0, 320.0 });
        m_updatedArmPosMap.put("ACQUIRE_DOUBLE_SUBSTATION_CONE", new double[] { 51.0, 2.0, 193.5 });
        m_updatedArmPosMap.put("ACQUIRE_DOUBLE_SUBSTATION_CUBE", new double[] { 47.1, 2.0, 203.7 });

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
        m_targetPosition = position;
    }

    /**
     * Runs the motor to a place with position PID & feedForward
     * 
     * @param position
     *            the encoder position to run it to
     * @param feedForward
     *            The return of {@code ElevatorFeedForward.calculate();}
     */
    public void runToPosition(double position, double feedForward) {
        m_pid.setReference(position, CANSparkMax.ControlType.kPosition, 0, feedForward);
        m_targetPosition = position;
    }

    public boolean atTargetPosition() {
        return getError() < 0.2;
    }

    public BooleanSupplier atTargetPositionSupplier() {
        return (() -> atTargetPosition());
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
     * 
     * @return the target position of the arm in rotations
     */
    public double getTargetPosition() {
        return m_targetPosition;
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return new InstantCommand(() -> {
        });
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a
     * digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
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

    public abstract Command changePositionCommand(double delta);

    @Override
    public void periodic() {
    }
}
