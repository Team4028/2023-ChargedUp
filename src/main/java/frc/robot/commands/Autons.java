// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auton.BeakAutonCommand;
import frc.robot.subsystems.Vision;
import frc.robot.utilities.drive.BeakDrivetrain;
import frc.robot.utilities.drive.Trajectories;

/** Stores all autonomous routines and helper functions. */
public class Autons {
    // Global Subsystems
    private final BeakDrivetrain m_drivetrain;
    private final Vision m_aprilTagVision;
    private final Vision m_gamePieceVision;

    public Autons(BeakDrivetrain drivetrain, Vision aprilTagVision, Vision gamePieceVision) {
        m_drivetrain = drivetrain;
        m_aprilTagVision = aprilTagVision;
        m_gamePieceVision = gamePieceVision;
    }

    public Command TwoPieceTopAcquire() {
        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, Trajectories.TwoPieceAcquirePiece(m_drivetrain),
                m_drivetrain.getTrajectoryCommand(Trajectories.TwoPieceAcquirePiece(m_drivetrain))
                // new WaitCommand(0.2),
                // m_drivetrain.generatePath(() -> m_gamePieceVision.getTargetPose(m_drivetrain.getPoseMeters(),
                //         new Transform3d(new Translation3d(Units.inchesToMeters(10.),
                //                 Units.inchesToMeters(-0.), 0.),
                //                 new Rotation3d()))));
        );

        return cmd.resetPoseAndRun();
    }

    public Command TwoPieceTopScore() {
        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, Trajectories.TwoPieceScorePiece(m_drivetrain),
                m_drivetrain.getTrajectoryCommand(Trajectories.TwoPieceScorePiece(m_drivetrain))
                // new WaitCommand(0.2),
                // m_drivetrain.generatePath(() -> m_gamePieceVision.getTargetPose(m_drivetrain.getPoseMeters(),
                //         new Transform3d(new Translation3d(Units.inchesToMeters(10.),
                //                 Units.inchesToMeters(-0.), 0.),
                //                 new Rotation3d()))));
        );

        return cmd.resetPoseAndRun();
    }

    public Command TwoPieceTop() {
        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, Trajectories.TwoPieceAcquirePiece(m_drivetrain),
                m_drivetrain.getTrajectoryCommand(Trajectories.TwoPieceAcquirePiece(m_drivetrain)),
                new WaitCommand(0.2),
                m_drivetrain.getTrajectoryCommand(Trajectories.TwoPieceScorePiece(m_drivetrain)));

        return cmd.resetPoseAndRun();
    }

    public Command JPath1() {
        // Pose2d desiredPose;

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, Trajectories.JPath1(m_drivetrain),
                m_drivetrain.getTrajectoryCommand(Trajectories.JPath1(m_drivetrain)),
                new WaitCommand(0.1),
                m_drivetrain.generatePath(() -> m_aprilTagVision.getTargetPose(m_drivetrain.getPoseMeters(),
                        new Transform3d(new Translation3d(Units.inchesToMeters(54.),
                                Units.inchesToMeters(-0.), 0.),
                                new Rotation3d()))) // 54 inches away from target
        );

        return cmd.resetPoseAndRun();
    }
}
