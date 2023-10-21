// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.beaklib.drive.swerve.BeakSwerveDrivetrain;

public class XDrive extends Command {
    private final BeakSwerveDrivetrain m_drivetrain;

    /** Creates a new XDrive. */
    public XDrive(BeakSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_drivetrain.setModuleStates(new SwerveModuleState[] {
                new SwerveModuleState(0., Rotation2d.fromDegrees(45.0)),
                new SwerveModuleState(0., Rotation2d.fromDegrees(-45.0)),
                new SwerveModuleState(0., Rotation2d.fromDegrees(-45.0)),
                new SwerveModuleState(0., Rotation2d.fromDegrees(45.0))
        });
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
