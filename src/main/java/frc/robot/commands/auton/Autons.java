// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.beaklib.drive.BeakDrivetrain;
import frc.robot.commands.chassis.AddVisionMeasurement;
import frc.robot.commands.chassis.ResetPoseToVision;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.arms.LowerArm;
import frc.robot.utilities.Trajectories;
import frc.robot.utilities.Trajectories.PathPosition;

/** Stores all autonomous routines and helper functions. */
public class Autons {
    // Global Subsystems
    private final BeakDrivetrain m_drivetrain;
    // private final LowerArm m_lowerArm;
    private final Vision m_frontAprilTagVision;
    private final Vision m_rearAprilTagVision;

    private final Map<String, Command> m_eventMap;

    // Subsystem & Event setup
    public Autons(
        BeakDrivetrain drivetrain,
        // LowerArm lowerArm,
        Vision frontAprilTagVision,
        Vision rearAprilTagVision) {
        m_drivetrain = drivetrain;
        // m_lowerArm = lowerArm;
        m_frontAprilTagVision = frontAprilTagVision;
        m_rearAprilTagVision = rearAprilTagVision;

        // The event map is used for PathPlanner's FollowPathWithEvents function.
        // Almost all pickup, scoring, and localization logic is done through events.
        m_eventMap = new HashMap<String, Command>();
        // m_eventMap.put("ArmScoring", new RunArm(45., m_lowerArm));
        // m_eventMap.put("ArmPickup", new RunArm(10., m_lowerArm));
        // m_eventMap.put("ArmRetract", new RunArm(2., m_lowerArm));

        m_eventMap.put("FrontLocalize", new AddVisionMeasurement(drivetrain, m_frontAprilTagVision));
        m_eventMap.put("RearLocalize", new AddVisionMeasurement(drivetrain, m_rearAprilTagVision));

        m_eventMap.put("FrontReset", new ResetPoseToVision(drivetrain, m_frontAprilTagVision));
        m_eventMap.put("RearReset", new ResetPoseToVision(drivetrain, m_rearAprilTagVision));
    }

    public BeakAutonCommand TwoPieceAcquire(PathPosition position) {
        // The Trajectories class lets you pass in a position, and that position will be
        // used to choose the path to load.
        PathPlannerTrajectory traj = Trajectories.TwoPieceAcquirePiece(m_drivetrain, position);

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, traj,
            // new RunArm(45., m_lowerArm),
            // new WaitCommand(0.2),
            m_drivetrain.getTrajectoryCommand(traj, m_eventMap)
        //
        );

        return cmd;
    }

    public BeakAutonCommand TwoPieceScore(PathPosition position) {
        PathPlannerTrajectory traj = Trajectories.TwoPieceScorePiece(m_drivetrain, position);

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, traj,
            m_drivetrain.getTrajectoryCommand(traj, m_eventMap)
        //
        );

        return cmd;
    }

    public BeakAutonCommand TwoPiece(PathPosition position) {
        // Acquire and Score already have existing paths, so the full two piece is
        // simply a combination of the two.
        BeakAutonCommand initialPath = TwoPieceAcquire(position);

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, initialPath.getInitialPose(),
            initialPath,
            TwoPieceScore(position)
        //
        );

        return cmd;
    }

    public BeakAutonCommand ThreePieceAcquire(PathPosition position) {
        PathPlannerTrajectory traj = Trajectories.ThreePieceAcquirePiece(m_drivetrain, position);

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, traj,
            m_drivetrain.getTrajectoryCommand(traj, m_eventMap)
        //
        );

        return cmd;
    }

    public BeakAutonCommand ThreePieceScore(PathPosition position) {
        PathPlannerTrajectory traj = Trajectories.ThreePieceScorePiece(m_drivetrain, position);

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, traj,
            m_drivetrain.getTrajectoryCommand(traj, m_eventMap)
        //
        );

        return cmd;
    }

    public BeakAutonCommand ThreePiece(PathPosition position) {
        // The Three Piece paths are made to continue off of the two piece path. Rather
        // than doing everything again, we simply run the two piece auton and continue
        // where we left off for the three piece paths.
        BeakAutonCommand initialPath = TwoPiece(position);

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, initialPath.getInitialPose(),
            initialPath,
            ThreePieceAcquire(position),
            ThreePieceScore(position)
        //
        );

        return cmd;
    }

    public BeakAutonCommand JPath1() {
        // example
        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, Trajectories.JPath1(m_drivetrain),
            m_drivetrain.getTrajectoryCommand(Trajectories.JPath1(m_drivetrain), m_eventMap),
            new WaitCommand(0.1),
            m_drivetrain.generatePath(() -> m_frontAprilTagVision.getTargetPose(m_drivetrain.getPoseMeters(),
                new Transform3d(new Translation3d(Units.inchesToMeters(54.),
                    Units.inchesToMeters(-0.), 0.),
                    new Rotation3d()))) // 54 inches away from target
        );

        return cmd;
    }
}
