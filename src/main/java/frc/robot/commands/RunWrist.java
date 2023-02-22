// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class RunWrist extends CommandBase {
    private Wrist m_wrist;
    private double m_position;
  /** Creates a new RunWrist. */
  public RunWrist(double position, Wrist wrist) {
    m_wrist = wrist;
    m_position = position;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.runWristToPosition(m_position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_wrist.getEncoderPosition() - m_wrist.getTargetPosition()) < 0.2;
  }
}
