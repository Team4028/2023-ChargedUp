// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.beaklib.drive.BeakDrivetrain;
import frc.robot.Constants;
import frc.robot.OneMechanism;
import frc.robot.OneMechanism.ScoringPositions;
import frc.robot.commands.chassis.AddVisionMeasurement;
import frc.robot.commands.chassis.AutoBalance;
import frc.robot.commands.chassis.ResetPoseToVision;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.arms.LowerArm;
import frc.robot.subsystems.arms.UpperArm;
import frc.robot.subsystems.manipulator.Gripper;
import frc.robot.subsystems.manipulator.Wrist;
import frc.robot.utilities.Trajectories;
import frc.robot.utilities.Trajectories.PathPosition;

/**
 * The Autons class stores all autonomous routines and helper functions.
 * 
 * <p>
 * Auton routines are typed as {@link BeakAutonCommand}s. Autons are stored as
 * functions that construct an inline sequential command routine. Pickup,
 * scoring, and localization logic is all done through PathPlanner's event
 * support, though with a few exceptions.
 */
public class Autons {
    // Auton Constants
    private static final double UPPER_ARM_OFFSET = 18.3;

    // Global Subsystems -- initialized in the constructor
    private final BeakDrivetrain m_drivetrain;

    private final LowerArm m_lowerArm;
    private final UpperArm m_upperArm;
    private final Wrist m_wrist;
    private final Gripper m_gripper;

    private final Vision m_frontAprilTagVision;
    private final Vision m_rearAprilTagVision;

    private final Map<String, Command> m_eventMap;

    private final BooleanSupplier m_armsAtPosition;

    // Subsystem & Event setup
    public Autons(
        BeakDrivetrain drivetrain,
        LowerArm lowerArm,
        UpperArm upperArm,
        Wrist wrist,
        Gripper gripper,
        Vision frontAprilTagVision,
        Vision rearAprilTagVision) {
        m_drivetrain = drivetrain;

        m_lowerArm = lowerArm;
        m_upperArm = upperArm;
        m_wrist = wrist;
        m_gripper = gripper;

        m_frontAprilTagVision = frontAprilTagVision;
        m_rearAprilTagVision = rearAprilTagVision;

        m_armsAtPosition = () -> (m_upperArm.atTargetPosition());

        // The event map is used for PathPlanner's FollowPathWithEvents function.
        // Almost all pickup, scoring, and localization logic is done through events.
        m_eventMap = new HashMap<String, Command>();
        if (Constants.PRACTICE_CHASSIS) {
            // TODO
            m_eventMap.put("HighScoring", OneMechanism.runArms(ScoringPositions.SCORE_HIGH_CONE));
            m_eventMap.put("MidScoring", OneMechanism.runArms(ScoringPositions.SCORE_MID_CONE));

            m_eventMap.put("CubePickup", OneMechanism.runArms(ScoringPositions.ACQUIRE_FLOOR_CUBE));
            m_eventMap.put("ConePickup", OneMechanism.runArms(ScoringPositions.ACQUIRE_FLOOR_CONE_UPRIGHT));

            m_eventMap.put("RunGripperIn", m_gripper.runMotorIn().until(m_gripper.atCurrentThresholdSupplier())
                .andThen(new InstantCommand(() -> m_gripper.beIdleMode())));
            m_eventMap.put("RunGripperOut", m_gripper.runMotorOut());
            m_eventMap.put("StopGripper", new InstantCommand(() -> m_gripper.beIdleMode()));

            m_eventMap.put("ArmRetract", OneMechanism.runArms(ScoringPositions.STOWED));
        }

        m_eventMap.put("FrontLocalize", new AddVisionMeasurement(drivetrain, m_frontAprilTagVision));
        m_eventMap.put("RearLocalize", new AddVisionMeasurement(drivetrain, m_rearAprilTagVision));

        m_eventMap.put("FrontReset", new ResetPoseToVision(drivetrain, m_frontAprilTagVision));
        m_eventMap.put("RearReset", new ResetPoseToVision(drivetrain, m_rearAprilTagVision));
    }

    // ================================================
    // ONE PIECE AUTONS
    // ================================================

    public BeakAutonCommand OnePieceBalance(PathPosition position) {
        // Acquire and Score already have existing paths, so the full two piece is
        // simply a combination of the two.
        PathPlannerTrajectory traj = Trajectories.OnePieceBalance(m_drivetrain, position);

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, traj.getInitialHolonomicPose(),
            m_drivetrain.getTrajectoryCommand(traj, m_eventMap),
            // BALANCE
            new WaitCommand(0.15),
            new AutoBalance(m_drivetrain, true)
        //
        );

        return cmd;
    }

    /**
     * The One Piece... IS REAL!
     * 
     * @return
     */
    public BeakAutonCommand OnePiece(PathPosition position) {
        // Score a piece, acquire a piece, then balance.
        BeakAutonCommand initialPath = TwoPieceAcquire(position);

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, initialPath.getInitialPose(),
            initialPath,
            // new WaitCommand(0.0),
            OnePieceBalance(position)
        //
        );

        return cmd;
    }

    // ================================================
    // TWO PIECE AUTONS
    // ================================================

    public BeakAutonCommand TwoPieceAcquire(PathPosition position) {
        // The Trajectories class lets you pass in a position, and that position will be
        // used to choose the path to load.
        PathPlannerTrajectory traj = Trajectories.TwoPieceAcquirePiece(m_drivetrain, position);

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, traj,
            new InstantCommand(() -> m_upperArm.setEncoderPosition(UPPER_ARM_OFFSET)),
            new WaitCommand(0.1),
            new InstantCommand(() -> m_upperArm.runArmVbus(-0.3)),
            new WaitCommand(0.07),

            OneMechanism.orangeModeCommand(),
            OneMechanism.runArms(ScoringPositions.SCORE_HIGH_CONE),//.until(m_armsAtPosition),
            new WaitCommand(0.2),

            m_gripper.runMotorOut().withTimeout(0.4),
            OneMechanism.runArms(ScoringPositions.STOWED).until(m_armsAtPosition),
            OneMechanism.purpleModeCommand(),
            m_drivetrain.getTrajectoryCommand(traj, m_eventMap)
        // new AddVisionMeasurement(m_drivetrain, m_rearAprilTagVision)
        //
        );

        return cmd;
    }

    public BeakAutonCommand TwoPieceScore(PathPosition position) {
        PathPlannerTrajectory traj = Trajectories.TwoPieceScorePiece(m_drivetrain, position);

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, traj,
            // new AddVisionMeasurement(m_drivetrain, m_rearAprilTagVision),
            m_drivetrain.getTrajectoryCommand(traj, m_eventMap),
            new WaitCommand(0.1),
            OneMechanism.runArms(ScoringPositions.SCORE_HIGH_CUBE).until(m_armsAtPosition),
            new WaitCommand(0.2),
            m_gripper.runMotorOutSoft().withTimeout(0.4),
            OneMechanism.runArms(ScoringPositions.STOWED)
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
            new WaitCommand(0.2),
            TwoPieceScore(position)
        //
        );

        return cmd;
    }

    public BeakAutonCommand TwoPieceBalance(PathPosition position) {
        // Acquire and Score already have existing paths, so the full two piece is
        // simply a combination of the two.
        BeakAutonCommand initialPath = TwoPieceBalance(position);

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, initialPath.getInitialPose(),
            initialPath,
            // Auto balance a few times
            new WaitCommand(0.5),
            new AutoBalance(m_drivetrain, true)
        //
        );

        return cmd;
    }

    // ================================================
    // THREE PIECE AUTONS
    // ================================================

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

    // ================================================
    // THIS IS NOT REAL
    // ================================================

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
