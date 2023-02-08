// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class CurrentZero extends CommandBase {
    private Arm m_armSubsystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     * @return
     */
    public CurrentZero(Arm arm) {
        m_armSubsystem = arm;
        // Use addRequirements() here to declare subsystem dependencies.

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_armSubsystem.runArm(-.1);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_armSubsystem.runArm(0.0);
        m_armSubsystem.setEncoderPosition(.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_armSubsystem.getMotorCurrent() >= 20;
    }
}
