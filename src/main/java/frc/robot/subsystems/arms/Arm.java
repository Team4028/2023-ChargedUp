// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arms;

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
    protected SparkMaxPIDController m_pid;
    protected double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, maxAcc, minVel, allowedErr;
    protected CANSparkMax m_motor;
    protected RelativeEncoder m_encoder;
    protected double m_pidPos, m_distanceToTravel = 0;
    public ElevatorFeedforward ffmodel;

    /**
     * the enum containing the desired positions of the arm
     */
    public enum ArmPositions {
        RETRACTED(0.5, 1.04166666667), // L: 2. U: 2.
        SCORE_MID(11, 27.0833333333), // L: 44. U: 52.
        SCORE_HIGH(14, 41.6666666667), // L: 56. U: 80.
        ACQUIRE_FLOOR_CUBE(2.25, 20), // L: 9. U: 40.
        ACQUIRE_FLOOR_TIPPED_CONE(2.25, 20), //L: 9. U: 40. 
        ACQUIRE_FLOOR_UPRIGHT_CONE(0.0,0.0),
        // We no longer care about these
        THIRTY(2.8325, 16.171875), // L: 11.33 U: 31.05
        SIXTY(4.8925, 27.92708333), // L: 19.57 U: 53.62
        NINETY(6.9525, 39.6875); // L: 27.81 U: 76.2

        public double lowerPosition;
        public double upperPosition;

        private ArmPositions(double lowerPosition, double upperPosition) {
            this.lowerPosition = lowerPosition;
            this.upperPosition = upperPosition;
        }
    }

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
        m_pid = m_motor.getPIDController();

        kP = .1;
        kI = 0.0;
        kD = 0.;
        kIz = 0.0;
        kFF = 0.0;
        kMaxOutput = .9;
        kMinOutput = -.9;
        // smart motion coefficients
        maxVel = 7000;
        maxAcc = 14000;
        allowedErr = 0.1;

        m_pid.setP(kP);
        m_pid.setI(kI);
        m_pid.setD(kD);
        m_pid.setIZone(kIz);
        m_pid.setFF(kFF);
        m_pid.setOutputRange(kMinOutput, kMaxOutput);

        m_motor.setIdleMode(IdleMode.kBrake);
        // Smart Motion
        m_pid.setSmartMotionMaxVelocity(maxVel, 0);
        m_pid.setSmartMotionMinOutputVelocity(minVel, 0);
        m_pid.setSmartMotionMaxAccel(maxAcc, 0);
        m_pid.setSmartMotionAllowedClosedLoopError(allowedErr, 0);

        m_motor.setOpenLoopRampRate(0.5);
        m_motor.setClosedLoopRampRate(0.1);
    }

    /**
     * runs the arm with raw vbus
     * @param speed the speed at which to run the arm
     */
    public void runArm(double speed) {
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
        m_pidPos = position;
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
        m_pidPos = position;
    }

    /**
     * 
     * @param nativeUntis the native unit (in this case rotations)
     * @return
     */
    abstract public double nativeUnitsToInches(double nativeUntis);

    /**
     * 
     * @param inches the inches to convert
     * @return the converted inches
     */
    abstract public double inchesToNativeUnits(double inches);

    public double getError() {
        return Math.abs(this.getEncoderPosition() - m_pidPos);
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
     * 
     * @return the target position of the arm in rotations 
     */
    public double getTargetPosition() {
        return m_pidPos;
    }

    /**
     * 
     * @return the double value of the target position of the arm in inches
     */
    abstract public double getTargetPositionInches();

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
     * @param dist the distance to travel
     */
    public void setDistanceToTravel(double dist) {
        m_distanceToTravel = dist;
    }

    public static Arm getInstance() {
        return null;
    }

}
