// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.beaklib.motor.BeakSparkMAX;
import frc.robot.OneMechanism;

public class Wrist extends SubsystemBase {
    private static final double kP = 0.06;
    private static final double kI = 0.0;
    private static final double kD = 0.4;
    private static final double kIz = 0;
    private static final double kFF = 0;

    private static final double kMaxOutput = 0.9;
    private static final double kMinOutput = -0.9;
    private static final double RAMP_RATE = 0.15;

    private static Wrist m_instance;

    private BeakSparkMAX m_motor;
    private SparkMaxAbsoluteEncoder m_absoluteEncoder;
    private SparkMaxPIDController m_pid;
    private double m_targetAngle;

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
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 18);

        m_absoluteEncoder = m_motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        m_absoluteEncoder.setPositionConversionFactor(360.0);
        // m_absoluteEncoder.setZeroOffset(275.);
        m_absoluteEncoder.setZeroOffset(198.5); // 186.11
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
     * Run the wrist.
     * 
     * @param vbus The percent output to run at.
     * @return A command that runs the motor, stopping it when cancelled.
     */
    public Command runMotor(double vbus) {
        return startEnd(
            () -> {
                m_motor.set(vbus);
            }, 
            () -> {
                m_motor.set(0.0);
                m_pid.setReference(getAbsoluteEncoderPosition(), ControlType.kPosition);
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
                m_targetAngle = angle;
            }).until(
                () -> Math.abs(getAbsoluteEncoderPosition() - angle) < 0.9);
    }

    /**@return A Command to hold the wrist at its current angle. Used after running open loop to stay put and not drop with gravity. */
    public Command holdWristAngle() {
        return runOnce(
            () -> {
                m_motor.set(0.0);
                m_targetAngle = getAbsoluteEncoderPosition();
                m_pid.setReference(m_targetAngle, ControlType.kPosition);
            }
        );
    }

    public Command changeAngleCommand(double delta) {
        return runOnce(() -> {
            // m_targetAngle += delta;
            OneMechanism.getScoringPosition().wristAngle += delta;
            m_pid.setReference(OneMechanism.getScoringPosition().wristAngle, ControlType.kPosition);
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
        SmartDashboard.putNumber("Wrist Pos", getAbsoluteEncoderPosition());
        SmartDashboard.putNumber("Wrist Targ", m_targetAngle);
        SmartDashboard.putNumber("Wrist RelPos", m_motor.getPositionNU());
        SmartDashboard.putNumber("Wrist Amps", m_motor.getOutputCurrent());
    }
}
