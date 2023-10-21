// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.beaklib.drive.BeakDrivetrain;

public class QuadraticAutoBalance extends Command {
    private final BeakDrivetrain m_drivetrain;

    private static final double PITCH_kP = 6.0;
    // private static final double PITCH_kI = 0.0;
    // private static final double PITCH_kD = 0.0;

    private static final double ROLL_kP = 0.05;
    // private static final double ROLL_kI = 0.0;
    // private static final double ROLL_kD = 0.0;

    /** Auto Balance, but AWESOME!!!! */
    public QuadraticAutoBalance(BeakDrivetrain drivetrain) {
        m_drivetrain = drivetrain;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        final double positionError = m_drivetrain.getGyroPitchRotation2d().getRadians();
        final double rollError = m_drivetrain.getGyroRollRotation2d().getRadians();

        m_drivetrain.drive(
            new ChassisSpeeds(
                Math.signum(positionError) * (positionError * positionError) * PITCH_kP,
                0.,
                0.)
        );
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(0, 0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
