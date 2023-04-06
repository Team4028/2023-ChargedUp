// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.lib.beaklib.drive.swerve.BeakSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotateDrivetrainContinuous extends PIDCommand {
    /** Creates a new RotateDrivetrainContinuous. */
    public RotateDrivetrainContinuous(Rotation2d goal, Supplier<Double> xInput, Supplier<Double> yInput,
        BeakSwerveDrivetrain drivetrain) {
        super(
            // The ProfiledPIDController used by the command
            drivetrain.createAutonThetaController(),
            // This should return the measurement
            () -> drivetrain.getRotation2d().getDegrees(),
            // This should return the goal (can also be a constant)
            () -> goal.getRadians(),
            // This uses the output
            (output) -> {
                // Use the output (and setpoint, if desired) here
                ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xInput.get(),
                    yInput.get(),
                    0., //output,
                    drivetrain.getRotation2d());
                
                drivetrain.drive(speeds);
            });
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);

        // Configure additional PID options by calling `getController` here.
        getController().enableContinuousInput(-Math.PI, Math.PI);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
