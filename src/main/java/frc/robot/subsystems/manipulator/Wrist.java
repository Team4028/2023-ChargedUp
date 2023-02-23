// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.motor.BeakSparkMAX;

public class Wrist extends SubsystemBase {
    private static Wrist m_instance;
    private BeakSparkMAX m_motor;

    private DutyCycleEncoder m_dutyCycleEncoder;

    private double m_targetPosition;
    private PIDController m_pidController;

    /** Creates a new Wrist. */
    public Wrist() {
        m_motor = new BeakSparkMAX(12);

        m_motor.restoreFactoryDefault();
        m_motor.setSmartCurrentLimit(25);
        m_motor.setInverted(false);

        m_dutyCycleEncoder = new DutyCycleEncoder(0);
        m_dutyCycleEncoder.setDutyCycleRange(1, 1024);

        m_pidController = new PIDController(0.2, 0, 0);
    }

    public void getAbsoluteEncoderPosition() {
        m_dutyCycleEncoder.getAbsolutePosition();
    }

    public Command runMotorUp() {
        return runOnce(
                () -> {
                    m_motor.set(0.4);
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
                    m_motor.set(-0.15);
                });
    }

    public Command runMotorToPosition(double position) {
        return runOnce(
                () -> {
                    m_targetPosition = position;
                });
    }

    public Command runToLowPosition() {
        return runOnce(
                () -> {
                    runMotorToPosition(0.);
                });
    }

    public Command runToMediumPosition() {
        return runOnce(
                () -> {
                    runMotorToPosition(10.);
                });
    }

    public Command runToHighPosition() {
        return runOnce(
                () -> {
                    runMotorToPosition(20.);
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
        m_motor.set(
                m_pidController.calculate(m_targetPosition - m_dutyCycleEncoder.getAbsolutePosition()));
    }
}
