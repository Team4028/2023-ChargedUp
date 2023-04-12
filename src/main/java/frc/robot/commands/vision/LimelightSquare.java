// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.lib.beaklib.drive.BeakDrivetrain;
import frc.robot.utilities.LimelightHelpers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LimelightSquare extends ProfiledPIDCommand {
    /** Creates a new LimelightSquare. */
    public LimelightSquare(DoubleSupplier xSupplier, DoubleSupplier ySupplier, BeakDrivetrain drivetrain) {
        super(
            // The ProfiledPIDController used by the command
            // drivetrain.createThetaController(),
            // new PIDController(
            // 1.0,
            // 0.,
            // 0.),
            // use pidcontroller
            new ProfiledPIDController(5., 0.0, 0,
                new TrapezoidProfile.Constraints(
                    drivetrain.getPhysics().maxAngularVelocity.getAsRadiansPerSecond() / 2., 1.0)),
            // This should return the measurement
            () -> Units.degreesToRadians(LimelightHelpers.getTX("")),
            // This should return the goal (can also be a constant)
            () -> 0.,
            // This uses the output
            (output, setpoint) -> {
                // Use the output (and setpoint, if desired) here
                drivetrain.drive(new ChassisSpeeds(
                    xSupplier.getAsDouble(),
                    ySupplier.getAsDouble(),
                    output + setpoint.velocity));
            });
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here.
        getController().enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drivetrain);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return Math.abs(LimelightHelpers.getTX("")) < (0.5);
        return false;
    }
}
