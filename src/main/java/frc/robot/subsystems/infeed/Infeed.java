// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.infeed;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Infeed extends SubsystemBase{
    private static Infeed m_instance;
    private TalonSRX m_motor;

    /** Creates a new Infeed. */
    public Infeed() {
        m_motor = new TalonSRX(11);
        
        m_motor.config_kP(0, 0.2);
        m_motor.config_kI(0,0);
        m_motor.config_kD(0, 0);
    }

    public Command runMotorIn() {
        return runOnce(
            () -> {
                m_motor.set(ControlMode.PercentOutput, 0.1);
            });
    }

    public Command stopMotor(){
        return runOnce(
            () -> {
                m_motor.set(ControlMode.PercentOutput, 0.0);
            });
    }
    
    public Command runMotorOut() {
        return runOnce(
            () -> {
             m_motor.set(ControlMode.PercentOutput, -0.1);
            });
    }

    public static Infeed getInstance() {
        if (m_instance == null) {
            m_instance = new Infeed();
        }
        return m_instance;
    }
}
