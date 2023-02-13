// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public abstract class Manipulator extends SubsystemBase {
    private Gripper m_gripper;
    private Wrist m_wrist;

    /** Creates a new Manipulator. */
    public Manipulator() {
        m_gripper = new Gripper();
        m_wrist = new Wrist();
    }

    public Command lowCube() {
        return runOnce(
            () -> {
                m_gripper.runToCubePosition();
                m_wrist.runToLowPosition();
            });
    }

    public Command lowCone() {
        return runOnce(
            () -> {
                m_gripper.runToConePosition();
                m_wrist.runToLowPosition();
            });
    }

    public Command midCube() {
        return runOnce(
            () -> {
                m_gripper.runToCubePosition();
                m_wrist.runToMediumPosition();
            });
    }

    public Command midCone() {
        return runOnce(
            () -> {
                m_gripper.runToConePosition();
                m_wrist.runToMediumPosition();
            });
    }

    public Command highCube() {
        return runOnce(
            () -> {
                m_gripper.runToCubePosition();
                m_wrist.runToHighPosition();
            });
    }

    public Command highCone() {
        return runOnce(
            () -> {
                m_gripper.runToConePosition();
                m_wrist.runToHighPosition();
            });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
