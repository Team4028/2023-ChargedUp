// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.beaklib.motor.BeakTalonSRX;
import frc.robot.OneMechanism;
import frc.robot.OneMechanism.GamePieceMode;

public class Gripper extends SubsystemBase {
    private static Gripper m_instance;
    private final BeakTalonSRX m_motor;

    private final double RUN_SPEED = 0.95;
    private final double SOFT_RUN_SPEED = 0.4;
    private final double HOLD_SPEED = 0.2; // 0.32;
    private final double IDLE_SPEED = 0.0; // 0.12;
    private final double HOLD_THRESHOLD = 45.0;
    private boolean m_hasGamePiece = false;

    private enum GripState {
        INFEED, //
        HOLD, //
        BEGIN_OUTFEED, //
        OUTFEED, //
        IDLE;
    }

    private GripState m_currentState;

    /** Creates a new Gripper. */
    public Gripper() {
        m_motor = new BeakTalonSRX(11);

        m_motor.configFactoryDefault();

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
        return runOnce(() -> m_hasGamePiece = false).andThen(
        runMotorInWithoutReset());
    }

    /** @return A Command to run the motor in. */
    public Command runMotorInWithoutReset() {
        return run(() -> {
            m_motor.set(RUN_SPEED);
            m_currentState = GripState.HOLD;
        });
    }

    /**
     * Stops the Motor.
     * <p>
     * This is deprecated and should not be used. Use beIdleMode() instead.
     * 
     * @return a command that does the above mentioned task
     */
    public Command stopMotor() {
        return runOnce(() -> {
            m_motor.set(0);
        });
    }

    /**
     * Yeet
     * 
     * @return A command that runs the motor out, stopping it when cancelled.
     */
    public Command runMotorOut() {
        return startEnd(() -> {
            m_currentState = GripState.IDLE;
            m_motor.set(-1.0 * RUN_SPEED);
        },
            () -> {
                m_motor.set(0.);
            });
    }

    /**
     * Less yeet
     * 
     * @return A command that runs the motor out softly, stopping it when cancelled.
     */
    public Command runMotorOutSoft() {
        return startEnd(() -> {
            m_currentState = GripState.IDLE;
            m_motor.set(-1.0 * SOFT_RUN_SPEED);
        },
            () -> {
                m_motor.set(0.);
            });
    }

    /**
     * Outfeeding that is sensitive to the current robot state; slow for cube and
     * fast for cone.
     */
    public Command modeSensitiveOutfeedCommand() {
        return new ConditionalCommand(
            runMotorOut(),
            runMotorOutSoft(),
            () -> OneMechanism.getGamePieceMode() == GamePieceMode.ORANGE_CONE);
    }

    /**
     * If the Gripper is in HOLD state, run the motor at the holding speed.
     * <p>
     * Else, run at the idle speed.
     */
    public void beIdleMode() {
        switch (m_currentState) {
            case HOLD:
                m_motor.set(HOLD_SPEED);
                break;
            case IDLE:
            default:
                m_motor.set(IDLE_SPEED);
                break;
        }
    }

    private boolean hasGamePiece() {
        return m_hasGamePiece;
    }

    public BooleanSupplier hasGamePieceSupplier() {
        return () -> hasGamePiece();
    }

    private boolean atCurrentThreshold() {
        if (m_motor.getSupplyCurrent() > HOLD_THRESHOLD && m_motor.get() > 0.) {
            OneMechanism.signalAcquisition();
            m_hasGamePiece = true;
            return true;
        } else {
            return false;
        }
    }

    public BooleanSupplier atCurrentThresholdSupplier() {
        return () -> atCurrentThreshold();
    }

    public static Gripper getInstance() {
        if (m_instance == null) {
            m_instance = new Gripper();
        }
        return m_instance;
    }

    public GripState getGripState() {
        return m_currentState;
    }

    public void periodic() {
        SmartDashboard.putNumber("Gripper Amps", m_motor.getSupplyCurrent());
        SmartDashboard.putString("Gripper Mode", getGripState().name());
        SmartDashboard.putBoolean("Gripper In", m_motor.get() >= RUN_SPEED);
        SmartDashboard.putBoolean("Gripper Out", m_motor.get() < 0.0);
        SmartDashboard.putBoolean("Gripper Hold", m_currentState == GripState.HOLD);
        SmartDashboard.putNumber("Gripper VBUS", m_motor.get());

        atCurrentThreshold();
    }
}