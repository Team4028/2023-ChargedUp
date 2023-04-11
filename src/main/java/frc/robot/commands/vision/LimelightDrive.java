// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.lib.beaklib.drive.BeakDrivetrain;
import frc.robot.OneMechanism;
import frc.robot.OneMechanism.ScoringPositions;
import frc.robot.utilities.LimelightHelpers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LimelightDrive extends PIDCommand {
    /** Creates a new LimelightDrive. */
    public LimelightDrive(BeakDrivetrain drivetrain) {
        super(
            // The controller that the command will use
            new PIDController(0.1, 0., 0.),
            // This should return the measurement
            () -> Units.degreesToRadians(LimelightHelpers.getTY("")),
            // This should return the setpoint (can also be a constant)
            () -> OneMechanism.getScoringPosition() == ScoringPositions.FLOOR_CUBE_SEEK ? -8.8 : -13.5,
            // This uses the output
            output -> {
                // Use the output here
                drivetrain.drive(new ChassisSpeeds(
                    -output,
                    0.,
                    0.));
            });
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here.
        addRequirements(drivetrain);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(getController().getPositionError()) < Units.degreesToRadians(0.5)
            || LimelightHelpers.getTY("") == 0. || LimelightHelpers.getTA("") < 2.;
    }
}
