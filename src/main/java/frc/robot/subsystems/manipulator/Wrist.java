// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.beaklib.motor.BeakSparkMAX;

public class Wrist extends SubsystemBase {

    /**
     * the desired positions of the wrist in degrees
     */
    public enum WristPositions {
        STOW(10), //
        INFEED_CUBE(240), //
        INFEED_TIPPED_CONE(240), //
        INFEED_UPRIGHT_CONE(0.0), //
        SCORE_HIGH(0.0), //
        SCORE_MID(0.0); //

        public double position;

        private WristPositions(double position) {
            this.position = position;
        }
    }

    private static Wrist m_instance;
    private BeakSparkMAX m_motor;
    private SparkMaxAbsoluteEncoder m_absoluteEncoder;
    private double m_targetPosition, kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    private SparkMaxPIDController m_pid;
    // private DutyCycleEncoder m_absEncoder;
    private PIDController my_pid;
    //private boolean pidEnabled;

    /** Creates a new Wrist. */
    public Wrist() {
        //pidEnabled = false;
        m_motor = new BeakSparkMAX(12);
        m_motor.restoreFactoryDefault();
        m_motor.setSmartCurrentLimit(25);
        m_motor.setInverted(false);
        m_motor.setIdleMode(IdleMode.kBrake);

        //m_absoluteEncoder = m_motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        m_absoluteEncoder = m_motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        //m_absoluteEncoder.setZeroOffset(0);
        m_pid = m_motor.getPIDController();
        m_pid.setFeedbackDevice(m_absoluteEncoder);

        //PID Constants
        kP = 0.4;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0;
        kMaxOutput = 0.9;
        kMinOutput = -0.9;
        m_pid.setP(kP);
        m_pid.setI(kI);
        m_pid.setD(kD);
        m_pid.setIZone(kIz);
        m_pid.setFF(kFF);
        m_pid.setOutputRange(kMinOutput, kMaxOutput);
    }

    // CONTROL METHODS

    /**
     * 
     * @return the absolute position of the absolute encoder in degrees
     */
    public double getAbsoluteEncoderPosition() {
        return Units.rotationsToDegrees(m_absoluteEncoder.getPosition());

    }

    /**
     * runs the wrist up
     * 
     * @return a command that does above mentioned task
     */
    public Command runMotorUp() {
        return runOnce(
            () -> {
                m_motor.set(-0.15);
            });
    }

    /**
     * stops the wrist
     * 
     * @return a command that does above mentioned task
     */
    public Command stopMotor() {
        return runOnce(
            () -> {
                m_motor.set(0);
            });
    }

    /**
     * runs the wrist down
     * 
     * @return a command that does above mentioned task
     */
    public Command runMotorDown() {
        return runOnce(
            () -> {
                m_motor.set(0.15);
            });
    }

    /**
     * runs the wrist to a set position
     * 
     * @param angle
     *            the angle to run the wrist to in degrees
     * @return a command that does above mentioned task
     */
    public Command runToAngle(double angle) {
        return runOnce(
            () -> {
                m_pid.setReference(angle, ControlType.kPosition);
            });
    }

    public Command runToLowPosition() {
        return runOnce(
            () -> {
                runToAngle(0.);
            });
    }

    public Command runToMediumPosition() {
        return runOnce(
            () -> {
                runToAngle(10.);
            });
    }

    public Command runToHighPosition() {
        return runOnce(
            () -> {
                runToAngle(20.);
            });
    }

    public static Wrist getInstance() {
        if (m_instance == null) {
            m_instance = new Wrist();
        }
        return m_instance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist abs pos", getAbsoluteEncoderPosition());
        SmartDashboard.putNumber("Raw Wrist", m_absoluteEncoder.getPosition());
    }
}
