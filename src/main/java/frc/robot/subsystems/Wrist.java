// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.motor.BeakSparkMAX;

public class Wrist extends SubsystemBase {
    public enum wristPositions{
        CUBE(0.0), //TODO: FILL IN COOEFS
        CONE(0.0);
        private double position;
        private wristPositions(double position){
            this.position = position;
        }
    }
    private SparkMaxPIDController m_pid;
    private double kP,kI,kD,kIz,kFF,kMaxOutput,kMinOutput;
    private BeakSparkMAX m_motor;
    private static Wrist m_instance;
  /** Creates a new Wrist. */
  public Wrist() {
    m_motor = new BeakSparkMAX(12);
    m_pid = m_motor.getPIDController();
    //PID Constants
    kP = 0;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput=.9;
    kMinOutput=-.9;
    m_pid.setP(kP);
    m_pid.setI(kI);
    m_pid.setD(kD);
    m_pid.setIZone(kIz);
    m_pid.setFF(kFF);
    m_pid.setOutputRange(kMinOutput, kMaxOutput);
  }

  public Command runWrist(double speed){
    return runOnce(()->{
        m_motor.set(speed);
    });
  }

  public void runWristToPosition(double position){
        m_pid.setReference(position, ControlType.kPosition);
  }

  public void runWristCube(){
    runWristToPosition(wristPositions.CUBE.position);
  }

  public void runWristCone(){
    runWristToPosition(wristPositions.CONE.position);
  }

  public static Wrist getInstance(){
    if(m_instance==null){
        m_instance=new Wrist();
    }
    return m_instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
