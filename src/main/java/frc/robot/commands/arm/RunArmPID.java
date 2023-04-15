// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OneMechanism;
import frc.robot.subsystems.arms.Arm;

public class RunArmPID extends CommandBase {
    private Arm m_arm;
    private double m_position;

    /** Creates a new RunArm. */
    public RunArmPID(double position, Arm arm) {
        m_position = position;
        m_arm = arm;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_arm.setTargetPosition(m_position);
        m_arm.runToPosition(m_position);

        m_arm.setDistanceToTravel(Math.abs(m_position - m_arm.getEncoderPosition()));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(m_arm.getEncoderPosition() - m_arm.getTargetPosition()) < 0.4;
    }
}
