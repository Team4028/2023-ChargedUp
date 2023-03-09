// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.beaklib.motor.BeakSparkMAX;

public class Wrist extends SubsystemBase {
    private static final double kP = 0.05;
    private static final double kI = 0;
    private static final double kD = 0.4;
    private static final double kIz = 0;
    private static final double kFF = 0;

    private static final double kMaxOutput = 0.4;
    private static final double kMinOutput = -0.4;
    private static final double RAMP_RATE = 0.15;

    private static Wrist m_instance;

    private BeakSparkMAX m_motor;
    private SparkMaxAbsoluteEncoder m_absoluteEncoder;
    private SparkMaxPIDController m_pid;

    // encoder in RIO:
    // private DutyCycleEncoder m_absEncoder;
    // private PIDController my_pid;
    //private boolean pidEnabled;

    /** Creates a new Wrist. */
    public Wrist() {
        //pidEnabled = false;
        m_motor = new BeakSparkMAX(12);
        m_motor.restoreFactoryDefault();
        m_motor.setSmartCurrentLimit(25);
        m_motor.setInverted(true);
        m_motor.setIdleMode(IdleMode.kCoast);

        m_absoluteEncoder = m_motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        m_absoluteEncoder.setPositionConversionFactor(360.0);
        m_absoluteEncoder.setZeroOffset(20);
        m_absoluteEncoder.setInverted(false);
        m_pid = m_motor.getPIDController();
        m_pid.setFeedbackDevice(m_absoluteEncoder);

        m_pid.setP(kP);
        m_pid.setI(kI);
        m_pid.setD(kD);
        m_pid.setIZone(kIz);
        m_pid.setFF(kFF);

        m_pid.setOutputRange(kMinOutput, kMaxOutput);

        m_motor.setClosedLoopRampRate(RAMP_RATE);
        m_motor.burnFlash();
    }

    // CONTROL METHODS

    /**
     * 
     * @return the absolute position of the absolute encoder in degrees
     */
    public double getAbsoluteEncoderPosition() {
        return m_absoluteEncoder.getPosition();
    }

    /**
     * runs the wrist up
     * 
     * @return a command that does above mentioned task
     */
    public Command runMotorUp() {
        return runOnce(
            () -> {
                m_motor.set(0.15);
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
                m_motor.set(-0.15);
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
        return run(
            () -> {
                m_pid.setReference(angle, ControlType.kPosition);
            }).until(
                () -> Math.abs(getAbsoluteEncoderPosition() - angle) < 1.0);
    }

    /**@return A Command to hold the wrist at its current angle. Used after running open loop to stay put and not drop with gravity. */
    public Command holdWristAngle() {
        return runOnce(
            () -> {
                m_motor.set(0.0);
                runToAngle(getAbsoluteEncoderPosition());
            }
        );
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
        SmartDashboard.putNumber("Wrist AppliedOutput", m_motor.getAppliedOutput());
        SmartDashboard.putNumber("Wrist Relative Position", m_motor.getPositionNU());
        SmartDashboard.putNumber("Wrist Current Output", m_motor.getOutputCurrent());
    }
}
