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
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Gripper {
    private TalonSRX m_motor;
    private Solenoid m_solenoid;

    /** Creates a new Gripper. */
    public Gripper() {
        m_motor = new TalonSRX(11);
        m_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        
        m_motor.config_kP(0, 0.2);
        m_motor.config_kI(0,0);
        m_motor.config_kD(0, 0);
    }

    public void toggle() {
        m_solenoid.toggle();
    }

    public void runMotorIn() {
        m_motor.set(ControlMode.PercentOutput, 0.3);
    }

    public void runMotorOut() {
        m_motor.set(ControlMode.PercentOutput, -0.3);
    }

    public void runMotorToPosition(double position) {
        m_motor.set(ControlMode.Position, position);
    }

    public void runToCubePosition() {
        runMotorToPosition(10. * 4096);
    }
    
    public void runToConePosition() {
        runMotorToPosition(30. * 4096);
    }
}
