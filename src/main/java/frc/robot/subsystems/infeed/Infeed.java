// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.infeed;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.motor.BeakTalonSRX;

public class Infeed extends SubsystemBase {
    private static Infeed m_instance;
    private BeakTalonSRX m_motor;

    /** Creates a new Infeed. */
    public Infeed() {
        m_motor = new BeakTalonSRX(11);
    }

    public Command runMotorIn() {
        return runOnce(
                () -> {
                    m_motor.set(0.1);
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
                    m_motor.set(-0.1);
                });
    }

    public static Infeed getInstance() {
        if (m_instance == null) {
            m_instance = new Infeed();
        }
        return m_instance;
    }
}
