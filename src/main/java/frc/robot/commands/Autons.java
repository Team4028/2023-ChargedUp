// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auton.BeakAutonCommand;
import frc.robot.commands.chassis.AddVisionMeasurement;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.arms.LowerArm;
import frc.robot.utilities.drive.BeakDrivetrain;
import frc.robot.utilities.drive.Trajectories;

/** Stores all autonomous routines and helper functions. */
public class Autons {
    // Global Subsystems
    private final BeakDrivetrain m_drivetrain;
    private final LowerArm m_lowerArm;
    private final Vision m_frontAprilTagVision;
    private final Vision m_rearAprilTagVision;

    private final Map<String, Command> m_eventMap;

    public enum TwoPiecePositions {
        BOTTOM, MIDDLE, TOP
    }

    public Autons(
        BeakDrivetrain drivetrain,
        LowerArm lowerArm,
        Vision frontAprilTagVision,
        Vision rearAprilTagVision) {
        m_drivetrain = drivetrain;
        m_lowerArm = lowerArm;
        m_frontAprilTagVision = frontAprilTagVision;
        m_rearAprilTagVision = rearAprilTagVision;

        m_eventMap = new HashMap<String, Command>();
        m_eventMap.put("ArmScoring", new RunArm(45., m_lowerArm));
        m_eventMap.put("ArmPickup", new RunArm(10., m_lowerArm));
        m_eventMap.put("ArmRetract", new RunArm(2., m_lowerArm));

        m_eventMap.put("FrontLocalize", new AddVisionMeasurement(drivetrain, m_frontAprilTagVision));
        m_eventMap.put("RearLocalize", new AddVisionMeasurement(drivetrain, m_rearAprilTagVision));
    }

    public BeakAutonCommand TwoPieceAcquire(TwoPiecePositions position) {
        PathPlannerTrajectory traj;
        switch (position) {
            case BOTTOM:
                traj = Trajectories.TwoPieceBottomAcquirePiece(m_drivetrain);
                break;
            case TOP:
                traj = Trajectories.TwoPieceTopAcquirePiece(m_drivetrain);
                break;
            default:
                System.err.println("Bruh you code is broken " + position);
                return null;
        }

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, traj,
            // new AddVisionMeasurement(m_drivetrain, m_frontAprilTagVision),
            new RunArm(45., m_lowerArm),
            new WaitCommand(0.2),
            m_drivetrain.getTrajectoryCommand(traj, m_eventMap).deadlineWith(
                // new RepeatCommand(new AddVisionMeasurement(m_drivetrain, m_frontAprilTagVision).alongWith(
                //     new AddVisionMeasurement(m_drivetrain, m_rearAprilTagVision))
                // //
                // )
            //
            )
        //
        );

        return cmd;
    }

    public BeakAutonCommand TwoPieceScore(TwoPiecePositions position) {
        PathPlannerTrajectory traj;
        switch (position) {
            case BOTTOM:
                traj = Trajectories.TwoPieceBottomScorePiece(m_drivetrain);
                break;
            case TOP:
                traj = Trajectories.TwoPieceTopScorePiece(m_drivetrain);
                break;
            default:
                System.err.println("Bruh you code is broken " + position);
                return null;
        }

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, traj,
            new RunArm(10., m_lowerArm),
            m_drivetrain.getTrajectoryCommand(traj, m_eventMap)
                // .deadlineWith(
                //     new RepeatCommand(new AddVisionMeasurement(m_drivetrain, m_frontAprilTagVision).alongWith(
                //         new AddVisionMeasurement(m_drivetrain, m_rearAprilTagVision))
                //     //
                //     )
                // //
                // )
        //
        );

        return cmd;
    }

    public BeakAutonCommand TwoPiece(TwoPiecePositions position) {
        BeakAutonCommand initialPath = TwoPieceAcquire(position);
        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, initialPath.getInitialPose(),
            initialPath,
            TwoPieceScore(position));

        return cmd;
    }

    public BeakAutonCommand JPath1() {
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
