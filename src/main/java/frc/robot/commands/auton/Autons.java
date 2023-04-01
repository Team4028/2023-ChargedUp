// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.beaklib.drive.BeakDrivetrain;
import frc.robot.Constants;
import frc.robot.OneMechanism;
import frc.robot.OneMechanism.ScoringPositions;
import frc.robot.commands.arm.RunArmPID;
import frc.robot.commands.chassis.AddVisionMeasurement;
import frc.robot.commands.chassis.QuadraticAutoBalance;
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

    private final BooleanSupplier m_upperArmStowed;
    private final BooleanSupplier m_upperArmExtended;

    private final Supplier<Command> m_stowCommand;
    private final Supplier<Command> m_cubeExtendCommand;
    private final Supplier<Command> m_coneExtendCommand;

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

        m_upperArmStowed = () -> (m_upperArm.getError() <= 16.0);
        m_upperArmExtended = () -> (m_upperArm.getError() <= 1.25);

        m_stowCommand = () -> new SequentialCommandGroup(
            new RunArmPID(ScoringPositions.STOWED.upperPosition.get(), m_upperArm)
                .alongWith(m_wrist.runToAngle(ScoringPositions.STOWED.wristAngle.get())).until(m_upperArmStowed),

            // Run the lower arm down but immediately end it.
            new RunArmPID(ScoringPositions.STOWED.lowerPosition.get(), m_lowerArm).until(() -> true));

        m_cubeExtendCommand = () -> new SequentialCommandGroup(
            new RunArmPID(ScoringPositions.SCORE_HIGH_CUBE.lowerPosition.get(), m_lowerArm)
                .until(() -> m_lowerArm.getError() < .40 * m_lowerArm.getDistanceToTravel()),

            // Run until the upper arm is "almost" there
            new RunArmPID(ScoringPositions.SCORE_HIGH_CUBE.upperPosition.get(), m_upperArm)
                .alongWith(m_wrist.runToAngle(ScoringPositions.SCORE_HIGH_CUBE.wristAngle.get())).until(m_upperArmExtended));

        m_coneExtendCommand = () -> new SequentialCommandGroup(
            new RunArmPID(ScoringPositions.SCORE_HIGH_CONE.lowerPosition.get(), m_lowerArm)
                .until(() -> m_lowerArm.getError() < .40 * m_lowerArm.getDistanceToTravel()),

            // Run until the upper arm is "almost" there
            new RunArmPID(ScoringPositions.SCORE_HIGH_CONE.upperPosition.get(), m_upperArm)
                .alongWith(m_wrist.runToAngle(ScoringPositions.SCORE_HIGH_CONE.wristAngle.get())).until(m_upperArmExtended));

        // The event map is used for PathPlanner's FollowPathWithEvents function.
        // Almost all pickup, scoring, and localization logic is done through events.
        m_eventMap = new HashMap<String, Command>();
        if (Constants.PRACTICE_CHASSIS) {
            m_eventMap.put("CubeHigh", m_cubeExtendCommand.get());
            m_eventMap.put("ConeHigh", m_coneExtendCommand.get());

            m_eventMap.put("CubePrep", OneMechanism.runArms(ScoringPositions.AUTON_PREP_CUBE));
            m_eventMap.put("ConePrep", OneMechanism.runArms(ScoringPositions.AUTON_PREP_CONE));

            m_eventMap.put("CubePickup", OneMechanism.runArms(ScoringPositions.ACQUIRE_FLOOR_CUBE));
            m_eventMap.put("ConePickup", OneMechanism.runArms(ScoringPositions.ACQUIRE_FLOOR_CONE_UPRIGHT));

            m_eventMap.put("RunGripperIn", m_gripper.runMotorIn());
            m_eventMap.put("RunGripperOut", m_gripper.runMotorOut());
            m_eventMap.put("StopGripper", new InstantCommand(() -> m_gripper.beIdleMode()));

            m_eventMap.put("ArmRetract", OneMechanism.runArms(ScoringPositions.STOWED));

            m_eventMap.put("RunGripperSmart", m_gripper.runMotorIn().until(m_gripper.atCurrentThresholdSupplier()));
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
        PathPlannerTrajectory traj = Trajectories.OnePieceBalance(m_drivetrain, position);

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, traj.getInitialHolonomicPose(),
            m_drivetrain.getTrajectoryCommand(traj, m_eventMap),
            // BALANCE
            new WaitCommand(0.15),
            new QuadraticAutoBalance(m_drivetrain)
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
            new InstantCommand(() -> OneMechanism.setScoreMode(true)),
            new InstantCommand(() -> OneMechanism.setClimbMode(false)),
            OneMechanism.orangeModeCommand(),

            new InstantCommand(() -> m_upperArm.setEncoderPosition(UPPER_ARM_OFFSET)),
            new WaitCommand(0.1),
            new InstantCommand(() -> m_upperArm.runArmVbus(-0.3)),
            new WaitCommand(0.07),

            m_coneExtendCommand.get(),
            m_gripper.runMotorOut().withTimeout(0.4),

            m_stowCommand.get(),

            new InstantCommand(() -> OneMechanism.setScoreMode(false)),
            OneMechanism.purpleModeCommand(),
            m_drivetrain.getTrajectoryCommand(traj, m_eventMap)
        //
        );

        return cmd;
    }

    public BeakAutonCommand TwoPieceScore(PathPosition position) {
        PathPlannerTrajectory traj = Trajectories.TwoPieceScorePiece(m_drivetrain, position);

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, traj,
            new InstantCommand(() -> OneMechanism.setScoreMode(true)),
            m_drivetrain.getTrajectoryCommand(traj, m_eventMap),

            // The arms start going to the high scoring position at the end of the path.
            new WaitUntilCommand(m_upperArmExtended),
            m_gripper.runMotorOutSoft().withTimeout(0.4),

            m_stowCommand.get(),
            new InstantCommand(() -> OneMechanism.setScoreMode(false))
        //
        );

        return cmd;
    }

    public BeakAutonCommand TwoPiece(PathPosition position, boolean balance) {
        // Acquire and Score already have existing paths, so the full two piece is
        // simply a combination of the two.
        BeakAutonCommand initialPath = TwoPieceAcquire(position);

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, initialPath.getInitialPose(),
            initialPath,
            TwoPieceScore(position),
            balance ? TwoPieceBalance(position) : Commands.none()
        //
        );

        return cmd;
    }

    public BeakAutonCommand TwoPieceBalance(PathPosition position) {
        PathPlannerTrajectory traj = Trajectories.TwoPieceBalance(m_drivetrain, position);

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, traj.getInitialHolonomicPose(),
            new InstantCommand(() -> OneMechanism.setClimbMode(true)),

            m_drivetrain.getTrajectoryCommand(traj, m_eventMap),
            // balance :)
            new QuadraticAutoBalance(m_drivetrain)
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
            OneMechanism.orangeModeCommand(),
            m_drivetrain.getTrajectoryCommand(traj, m_eventMap)
        //
        );

        return cmd;
    }

    public BeakAutonCommand ThreePieceScore(PathPosition position) {
        PathPlannerTrajectory traj = Trajectories.ThreePieceScorePiece(m_drivetrain, position);

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, traj,
            new InstantCommand(() -> OneMechanism.setScoreMode(true)),
            m_drivetrain.getTrajectoryCommand(traj, m_eventMap),

            // The arms start going to the high scoring position at the end of the path.
            new WaitUntilCommand(m_upperArmExtended),
            // m_gripper.runMotorOut().withTimeout(0.4),

            m_stowCommand.get(),
            new InstantCommand(() -> OneMechanism.setScoreMode(false))
        //
        );

        return cmd;
    }

    public BeakAutonCommand ThreePiece(PathPosition position, boolean balance) {
        // The Three Piece paths are made to continue off of the two piece path. Rather
        // than doing everything again, we simply run the two piece auton and continue
        // where we left off for the three piece paths.
        BeakAutonCommand initialPath = TwoPiece(position, false);

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, initialPath.getInitialPose(),
            initialPath,
            ThreePieceAcquire(position),
            m_gripper.runMotorInWithoutReset().until(m_gripper.atCurrentThresholdSupplier()).until(m_gripper.hasGamePieceSupplier()).withTimeout(3.0),
            new WaitUntilCommand(m_gripper.hasGamePieceSupplier()),
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
