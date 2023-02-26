// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.motor.BeakTalonSRX;

public class Gripper extends SubsystemBase {
    private static Gripper m_instance;
    private BeakTalonSRX m_motor;

    /** Creates a new Gripper. */
    public Gripper() {
        m_motor = new BeakTalonSRX(11);
    }

    public Command runMotorIn() {
        return runOnce(
            () -> {
                m_motor.set(0.75);
            });
    }

    public Command stopMotor() {
        return runOnce(
            () -> {
                m_motor.set(0.0);
            });
    }

    public Command runMotorOut() {
        return runOnce(
            () -> {
                m_motor.set(-0.75);
            });
    }

    public static Gripper getInstance() {
        if (m_instance == null) {
            m_instance = new Gripper();
        }
        return m_instance;
    }
}
