// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.beaklib.motor.BeakTalonSRX;

public class Gripper extends SubsystemBase {
    private static Gripper m_instance;
    private final BeakTalonSRX m_motor;

    private final double RUN_SPEED = 0.85;
    private final double HOLD_SPEED = 0.32;
    private final double HOLD_THRESHOLD = 50.0;

    private enum GripState {
        INFEED, HOLD, BEGIN_OUTFEED, OUTFEED, IDLE
    }

    private GripState m_currentState;

    /** Creates a new Gripper. */
    public Gripper() {
        m_motor = new BeakTalonSRX(11);

        m_motor.config_kP(0, 0.2);
        m_motor.config_kI(0, 0);
        m_motor.config_kD(0, 0);

        m_motor.configPeakCurrentLimit(40);
        m_motor.configPeakCurrentDuration(100);
        m_motor.configContinuousCurrentLimit(20);
        m_motor.enableCurrentLimit(true);
        m_currentState = GripState.IDLE;
    }

    /** @return A Command to run the motor in. */
    public Command runMotorIn() {
        return run(() -> {
            m_motor.set(RUN_SPEED);
        });
    }

    /**
     * stops the motor
     * 
     * @return a command that does the above mentioned task
     */
    public Command stopMotor() {
        return runOnce(() -> {
            m_motor.set(0);
        });
    }

    /**
     * runs the motor out
     * 
     * @return a command that does the above mentioned task
     */
    public Command runMotorOut() {
        return run(() -> {
            m_motor.set(-1.0 * RUN_SPEED);
        });
    }

    public void holdGamePiece() {
        m_motor.set(HOLD_SPEED);
    }

    public BooleanSupplier atCurrentThreshold() {
        return () -> m_motor.getSupplyCurrent() > HOLD_THRESHOLD;
    }

    public static Gripper getInstance() {
        if (m_instance == null) {
            m_instance = new Gripper();
        }
        return m_instance;
    }

    public void periodic() {
        SmartDashboard.putNumber("GripperAmps", m_motor.getSupplyCurrent());
    }
}