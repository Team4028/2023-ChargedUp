// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.units.Distance;

/**
 * The upper Argos Arm
 */
public abstract class Arm extends SubsystemBase {
    protected SparkMaxPIDController m_pid;
    protected double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, maxAcc, minVel, allowedErr;
    protected CANSparkMax m_motor;
    protected RelativeEncoder m_encoder;
    protected double m_pidPos, m_distanceToTravel = 0;

    public enum ArmPositions {
        RETRACTED(7., 7.),
        SCORE_MID(22, 52),
        SCORE_HIGH(28, 80),
        ACQUIRE_FLOOR(9, 45),
        THIRTY(11.33*2, 17),
        SIXTY(19.57*2, 34),
        NINETY(25.31*2, 51);

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
    protected void initArm()
    {
        m_encoder = m_motor.getEncoder();
        m_motor.setSmartCurrentLimit(40);
        m_pid = m_motor.getPIDController();

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

    public void runArm(double speed) {
        m_motor.set(speed);
    }

    public double getMotorCurrent() {
        return m_motor.getOutputCurrent();
    }

    public void runToPosition(double position) {
        m_pid.setReference(position, CANSparkMax.ControlType.kSmartMotion);
        m_pidPos = position;
        m_distanceToTravel = Math.abs(position - getEncoderPosition());        
    }

    public double getError() {
        return Math.abs(this.getEncoderPosition() - m_pidPos);
    }

    public void zeroEncoder() {
        m_encoder.setPosition(0.0);
    }

    public double getEncoderPosition() {
        return m_encoder.getPosition();
    }

    public double getTargetPosition() {
        return m_pidPos;
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return new InstantCommand(() -> {});
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

    public double getDistanceToTravel() {
        return m_distanceToTravel;
    }

    public static Arm getInstance() {return null;}

}
