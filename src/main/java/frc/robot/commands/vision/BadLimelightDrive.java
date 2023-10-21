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
import frc.robot.subsystems.manipulator.Gripper;
import frc.robot.utilities.LimelightHelpers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BadLimelightDrive extends PIDCommand {
    private Gripper m_gripper;

    /** Creates a new LimelightDrive. */
    public BadLimelightDrive(Gripper gripper, BeakDrivetrain drivetrain) {
        super(
            // The controller that the command will use
            new PIDController(0.1, 0., 0.),
            // This should return the measurement
            () -> Units.degreesToRadians(LimelightHelpers.getTY("")),
            // This should return the setpoint (can also be a constant)
            () -> OneMechanism.getScoringPosition() == ScoringPositions.FLOOR_CUBE_SEEK ? -8.8 : -16.0,
            // This uses the output
            output -> {
                // Use the output here
                drivetrain.drive(new ChassisSpeeds(
                    -output * 0.5,
                    0.,
                    0.));
            });
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here.
        addRequirements(drivetrain, gripper);

        m_gripper = gripper;
    }

    @Override
    public void initialize() {
        super.initialize();

        m_gripper.runMotorIn();
    }

    @Override
    public void execute() {
        super.execute();
        m_gripper.runMotorInWithoutReset();
        m_gripper.atCurrentThreshold();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return (Math.abs(getController().getPositionError()) < Units.degreesToRadians(0.5)
        //     || LimelightHelpers.getTY("") == 0. || LimelightHelpers.getTA("") < 2.) && m_gripper.hasGamePieceSupplier().getAsBoolean();
        return !LimelightHelpers.getTV("") || m_gripper.hasGamePiece();
    }
}
