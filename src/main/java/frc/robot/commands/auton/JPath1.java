// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Vision;
import frc.robot.utilities.drive.BeakDrivetrain;
import frc.robot.utilities.drive.Trajectories;

public class JPath1 extends BeakAutonCommand {
    private Pose2d m_desiredAprTagPose;
    private Transform2d m_poseDiff;

    /** Creates a new TestPath. */
    public JPath1(Vision vision, BeakDrivetrain drivetrain) {
        super.addCommands(
                // drivetrain.getTrajectoryCommand(Trajectories.JPath1(drivetrain)),
                // // new WaitCommand(0.1),
                // new InstantCommand(() -> m_desiredAprTagPose =
                // vision.getTargetPose(drivetrain.getPoseMeters(),
                // new Transform3d(new Translation3d(Units.inchesToMeters(54.),
                // Units.inchesToMeters(-0.), 0.),
                // new Rotation3d()))), // This gets the position of a point square to the
                // target, and 54 inches away.
                // new WaitCommand(0.1),
                new NewNewGeneratePath(Trajectories.JPath1(drivetrain),
                        () -> vision.getTargetPose(drivetrain.getPoseMeters(),
                                new Transform3d(
                                        new Translation3d(Units.inchesToMeters(54.), Units.inchesToMeters(-0.), 0.),
                                        new Rotation3d())),
                        drivetrain)
                // new InstantCommand(() -> {
                //     Field2d field = new Field2d();
                //     field.setRobotPose(m_desiredAprTagPose);
                //     SmartDashboard.putData("AprilTag Pose", field);
                // })
                // new GeneratePath(() -> m_desiredAprTagPose, drivetrain), // The actual path generation/running
                // new WaitCommand(0.1),
                // new InstantCommand(() -> m_poseDiff = drivetrain.getPoseMeters().minus(m_desiredAprTagPose)),
                // new InstantCommand(() -> SmartDashboard.putNumber("X error", m_poseDiff.getX())),
                // new InstantCommand(() -> SmartDashboard.putNumber("Y error", m_poseDiff.getY())),
                // new InstantCommand(
                //         () -> SmartDashboard.putNumber("Theta error", m_poseDiff.getRotation().getDegrees())),
                // new RotateDrivetrainToAngle(() -> m_desiredAprTagPose.getRotation(), drivetrain, false) // verify the
                //                                                                                         // angle
        );
        super.setInitialPose(Trajectories.JPath1(drivetrain));
    }
}
