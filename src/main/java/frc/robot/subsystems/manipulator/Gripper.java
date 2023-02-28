// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase{
    private TalonSRX m_motor;
    private static Gripper m_instance;

    /** Creates a new Gripper. */
    public Gripper() {
        m_motor = new TalonSRX(11);

        m_motor.config_kP(0, 0.2);
        m_motor.config_kI(0, 0);
        m_motor.config_kD(0, 0);

        m_motor.configPeakCurrentLimit(30);
        m_motor.configPeakCurrentDuration(100);
        m_motor.configContinuousCurrentLimit(20);
        m_motor.enableCurrentLimit(true);
    }

    /**
     * runs the motor in
     * @return a command that does the above mentioned task
     */
    public Command runMotorIn() {
        return runOnce(()->{
            m_motor.set(ControlMode.PercentOutput, 0.7);
        });
    }

    /**
     * stops the motor
     * @return a command that does the above mentioned task
     */
    public Command stopMotor(){
        return runOnce(()->{
            m_motor.set(ControlMode.PercentOutput, 0);
        });
    }

    /**
     * runs the motor out
     * @return a command that does the above mentioned task
     */
    public Command runMotorOut() {
        return runOnce(()->{
            m_motor.set(ControlMode.PercentOutput, -0.7);
        });
    }

    public static Gripper getInstance(){
        if(m_instance==null){
            m_instance = new Gripper();
        }
        return m_instance;
    }
}
