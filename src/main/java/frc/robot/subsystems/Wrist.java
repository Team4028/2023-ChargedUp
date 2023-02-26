// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.motor.BeakSparkMAX;

public class Wrist extends SubsystemBase {

    public enum WristPositions {
        STOW(0.0), //
        INFEED_CUBE(0.0), //
        INFEED_CONE(0.0), //
        SCORE_HIGH(0.0), //
        SCORE_MID(0.0);

        public double position;

        private WristPositions(double position) {
            this.position = position;
        }
    }

    private static Wrist m_instance;
    private BeakSparkMAX m_motor;
    private SparkMaxAbsoluteEncoder m_absoluteEncoder;
    private DutyCycleEncoder m_absEnc;
    private double m_targetPosition, kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    private SparkMaxPIDController m_pid;
    private PIDController my_pid;

    /** Creates a new Wrist. */
    public Wrist() {
        m_motor = new BeakSparkMAX(12);
        m_absEnc = new DutyCycleEncoder(1);
        m_motor.restoreFactoryDefault();
        m_motor.setSmartCurrentLimit(50);
        m_motor.setInverted(true);

        m_absoluteEncoder = m_motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        m_absoluteEncoder.setZeroOffset(0);

        m_pid = m_motor.getPIDController();

        // PID Constants
        kP = 0;
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
        my_pid = new PIDController(kP, kI, kD);
    }

    // CONTROL METHODS

    public double getAbsoluteEncoderPosition() {
        return m_absoluteEncoder.getPosition() * 360.;

    }

    public Command runMotorUp() {
        return runOnce(
            () -> {
                m_motor.set(0.1);
            });
    }

    public Command stopMotor() {
        return runOnce(
            () -> {
                m_motor.set(0);
            });
    }

    public Command runMotorDown() {
        return runOnce(
            () -> {
                m_motor.set(-0.1);
            });
    }

    public Command runToPosition(double position) {
        return runOnce(
            () -> {
                m_targetPosition = position;
                MathUtil.clamp(my_pid.calculate(m_absEnc.getAbsolutePosition(), position), -0.5, 0.5);
            });
    }

    public Command runToLowPosition() {
        return runOnce(
            () -> {
                runToPosition(0.);
            });
    }

    public Command runToMediumPosition() {
        return runOnce(
            () -> {
                runToPosition(10.);
            });
    }

    public Command runToHighPosition() {
        return runOnce(
            () -> {
                runToPosition(20.);
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
        //m_pid.setReference(m_targetPosition, ControlType.kPosition);
        
        //m_motor.set(MathUtil.clamp(my_pid.calculate(m_absEnc.getAbsolutePosition(), m_targetPosition), -0.5, 0.5));
        
        SmartDashboard.putNumber("wristAbsPos", m_absEnc.getAbsolutePosition());
    }
}
