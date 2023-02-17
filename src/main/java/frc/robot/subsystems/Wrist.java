// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.motor.BeakSparkMAX;

public class Wrist extends SubsystemBase {
    private BeakSparkMAX m_motor;
    private static Wrist m_instance;
  /** Creates a new Wrist. */
  public Wrist() {
    m_motor=new BeakSparkMAX(12);
  }

  public Command runWrist(double speed){
    return runOnce(()->{
        m_motor.set(speed);
    });
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
