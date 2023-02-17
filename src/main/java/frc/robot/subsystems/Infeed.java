// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.motor.BeakTalonSRX;

public class Infeed extends SubsystemBase {
    private BeakTalonSRX m_motor;
    private static Infeed m_instance;
  /** Creates a new Infeed. */
  public Infeed() {
    m_motor=new BeakTalonSRX(11);
  }

  public Command runInfeed(double speed){
    return runOnce(()->{
        m_motor.set(speed);
    });
  }

  public static Infeed getInstance(){
    if(m_instance==null){
        m_instance=new Infeed();
    }
    return m_instance;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
