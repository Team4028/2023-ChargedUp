// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Wrist {
    private TalonSRX m_motor;

    /** Creates a new Wrist. */
    public Wrist() {
        m_motor = new TalonSRX(12);

        m_motor.config_kP(0, 0.2);
        m_motor.config_kI(0, 0);
        m_motor.config_kD(0, 0);
    }

    public void runMotorUp() {
        m_motor.set(ControlMode.PercentOutput, 0.2);
    }

    public void runMotorDown() {
        m_motor.set(ControlMode.PercentOutput, -0.2);
    }

    public void runMotorToPosition(double position) {
        m_motor.set(ControlMode.Position, position);
    }

    public void runToLowPosition() {
        runMotorToPosition(0. * 4096);
    }

    public void runToMediumPosition() {
        runMotorToPosition(10. * 4096);
    }

    public void runToHighPosition() {
        runMotorToPosition(20 * 4096);
    }
}
