// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kickstand;

import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.beaklib.motor.BeakSparkMAX;

public class Kickstand extends SubsystemBase {
    private final double RETRACTED_POSITION = 0.0;
    private final double ACTIVE_POSITION = 110.0;
    private BeakSparkMAX m_motor;
    private static Kickstand m_instance;
    private SparkMaxAbsoluteEncoder m_absoluteEncoder;
    private SparkMaxPIDController m_pid;

    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kIz = 0.0;
    private static final double kFF = 0.0;
    private static final double kMaxOutput = 0.4;
    private static final double kMinOutput = -0.4;

    /** Creates a new Kickstand. */
    public Kickstand() {
        m_motor = new BeakSparkMAX(21);
        m_absoluteEncoder = m_motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        m_absoluteEncoder.setPositionConversionFactor(360.0);
        m_pid = m_motor.getPIDController();
        m_pid.setFeedbackDevice(m_absoluteEncoder);

        m_pid.setP(kP);
        m_pid.setI(kI);
        m_pid.setD(kD);
        m_pid.setIZone(kIz);
        m_pid.setFF(kFF);
        m_pid.setOutputRange(kMinOutput, kMaxOutput);
    }

    /**
     * Runs the motor at the inputted vbus
     */
    public Command runMotor(double speed) {
        return runOnce(() -> {
            m_motor.set(speed);
        });
    }

    public void runToPosition(double degrees) {
        m_pid.setReference(degrees, ControlType.kPosition);
    }

    public double getAbsoluteEncoderPosition() {
        return m_absoluteEncoder.getPosition();
    }

    public static Kickstand getInstance() {
        if (m_instance == null) {
            m_instance = new Kickstand();
        }
        return m_instance;
    }

    public void activateKickstand() {
            runToPosition(ACTIVE_POSITION);
    }

    public void deacitvateKickstand() {
            runToPosition(RETRACTED_POSITION);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Kickstand Output Current", m_motor.getOutputCurrent());
        SmartDashboard.putNumber("Kickstand Absolute Position", getAbsoluteEncoderPosition());
    }
}
