// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ArmThirty extends CommandBase {
    private Arm m_upperArm=Arm.getInstance();
    private Arm2 m_lowerArm=Arm2.getInstance();
  /** Creates a new ArmThirty. */
  public ArmThirty() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_lowerArm.getEncoder()<11.33103){
        new InstantCommand(m_lowerArm::armThirty).alongWith(new WaitCommand(1).andThen(new InstantCommand(m_upperArm::armThirty)));
    } else{
        new InstantCommand(m_upperArm::armThirty).alongWith(new WaitCommand(1).andThen(new InstantCommand(m_lowerArm::armThirty)));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
