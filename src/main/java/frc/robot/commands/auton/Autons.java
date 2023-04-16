// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.beaklib.drive.swerve.BeakSwerveDrivetrain;
import frc.lib.beaklib.drive.swerve.BeakSwerveDrivetrain.SnapDirection;
import frc.robot.OneMechanism;
import frc.robot.OneMechanism.GamePieceMode;
import frc.robot.OneMechanism.ScoringPositions;
import frc.robot.commands.arm.RunArmPID;
import frc.robot.commands.chassis.AddVisionMeasurement;
import frc.robot.commands.chassis.SnapToAngle;
import frc.robot.commands.chassis.QuadraticAutoBalance;
import frc.robot.commands.chassis.ResetPoseToVision;
import frc.robot.commands.vision.LimelightSquare;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.arms.LowerArm;
import frc.robot.subsystems.arms.UpperArm;
import frc.robot.subsystems.manipulator.Gripper;
import frc.robot.subsystems.manipulator.Wrist;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.Trajectories;
import frc.robot.utilities.Trajectories.PathPart;
import frc.robot.utilities.Trajectories.PathPosition;

/**
 * The Autons class stores all autonomous routines and helper functions.
 * 
 * <p>
 * Auton routines are typed as {@link BeakAutonCommand}s. Autons are stored as
 * functions that construct an inline sequential command routine. Pickup,
 * scoring, and localization logic is all done through PathPlanner's event
 * support, though with a few exceptions.
 * </p>
 * 
 * <p>
 * I need to document all this architecture soon but here's a basic gist.
 * </p>
 * 
 * <p>
 * Auton paths are made and all named according to one scheme. Generally, there
 * will be discrete sets of patterns within these paths (i.e. top, middle,
 * bottom and score, acquire, balance). Thus, generic auton commands for each
 * individual "action" (for 2023, score, acquire, and balance) are created.
 * These generic commands can take in a position (bottom, middle, top).
 * Additional logic can be added in as well. To make the final autons, generic
 * paths are strung together, and sometimes additional logic is added to the
 * mix.
 * </p>
 */
public class Autons {
    // Auton Constants
    private static final double UPPER_ARM_OFFSET = 18.4;
    // private static final double LOWER_ARM_OFFSET = 0.86;

    private static final double[] CUBE_TWO_CROP = { -1, 0.57, -1, 0.81 };
    private static final double[] CUBE_THREE_CROP = { -0.57, 1, -1, 0.81 };

    // Global Subsystems -- initialized in the constructor
    private final BeakSwerveDrivetrain m_drivetrain;

    private final LowerArm m_lowerArm;
    private final UpperArm m_upperArm;
    private final Wrist m_wrist;
    private final Gripper m_gripper;

    private final Vision m_frontAprilTagVision;
    private final Vision m_rearAprilTagVision;

    // The "meat" of auton
    private final Map<String, Command> m_eventMap;

    private final BooleanSupplier m_upperArmStowed;
    private final BooleanSupplier m_upperArmExtended;

    private final Supplier<Command> m_stowCommand;
    private final Supplier<Command> m_cubeExtendCommand;
    private final Supplier<Command> m_coneExtendCommand;

    // Subsystem & Event setup
    public Autons(
        BeakSwerveDrivetrain drivetrain,
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

        m_upperArmStowed = () -> (m_upperArm.getError() <= 0.6 * m_upperArm.getDistanceToTravel());
        // m_upperArmExtended = () -> (m_upperArm.getError() <= 1.25);
        m_upperArmExtended = () -> (m_upperArm.getError() <= 0.04 * m_upperArm.getDistanceToTravel())
            && (m_lowerArm.getError() <= 0.6 * m_lowerArm.getDistanceToTravel());

        m_stowCommand = () -> OneMechanism.runArms(ScoringPositions.STOWED).until(m_upperArmStowed);
        // shouldnt be needed?
        // m_stowCommand = () -> new SequentialCommandGroup(
        // new RunArmPID(ScoringPositions.STOWED.upperPosition, m_upperArm)
        // .alongWith(m_wrist.runToAngle(ScoringPositions.STOWED.wristAngle)).until(m_upperArmStowed),

        // // Run the lower arm down but immediately end it.
        // new RunArmPID(ScoringPositions.STOWED.lowerPosition, m_lowerArm).until(() ->
        // true));

        m_cubeExtendCommand = () -> OneMechanism.runArms(ScoringPositions.SCORE_HIGH_CUBE);// .until(m_upperArmExtended);
        // shouldnt be needed?
        // m_cubeExtendCommand = () -> new SequentialCommandGroup(
        // new RunArmPID(ScoringPositions.SCORE_HIGH_CUBE.lowerPosition, m_lowerArm)
        // .until(() -> m_lowerArm.getError() < .40 * m_lowerArm.getDistanceToTravel()),

        // // Run until the upper arm is "almost" there
        // new RunArmPID(ScoringPositions.SCORE_HIGH_CUBE.upperPosition, m_upperArm)
        // .alongWith(m_wrist.runToAngle(ScoringPositions.SCORE_HIGH_CUBE.wristAngle)).until(m_upperArmExtended));

        m_coneExtendCommand = () -> OneMechanism.runArms(ScoringPositions.SCORE_HIGH_CONE).until(m_upperArmExtended);
        // m_coneExtendCommand = () -> new SequentialCommandGroup(
        // new RunArmPID(ScoringPositions.SCORE_HIGH_CONE.lowerPosition, m_lowerArm)
        // .until(() -> m_lowerArm.getError() < .40 * m_lowerArm.getDistanceToTravel()),

        // // Run until the upper arm is "almost" there
        // new RunArmPID(ScoringPositions.SCORE_HIGH_CONE.upperPosition, m_upperArm)
        // .alongWith(m_wrist.runToAngle(ScoringPositions.SCORE_HIGH_CONE.wristAngle)).until(m_upperArmExtended));

        // The event map is used for PathPlanner's FollowPathWithEvents function.
        // Almost all pickup, scoring, and localization logic is done through events.
        m_eventMap = new HashMap<String, Command>();

        m_eventMap.put("CubeHigh", m_cubeExtendCommand.get());
        m_eventMap.put("ConeHigh", m_coneExtendCommand.get());

        m_eventMap.put("CubeMid", OneMechanism.runArms(ScoringPositions.SCORE_MID_CUBE));
        m_eventMap.put("ConeMid", OneMechanism.runArms(ScoringPositions.SCORE_MID_CONE));

        m_eventMap.put("CubePrep", OneMechanism.runArms(ScoringPositions.AUTON_PREP_CUBE));
        m_eventMap.put("ConePrep", OneMechanism.runArms(ScoringPositions.AUTON_PREP_CONE));

        m_eventMap.put("CubePickup",
            OneMechanism.runArmsSimultaneouslyCommand(ScoringPositions.ACQUIRE_FLOOR_CUBE)
                .alongWith(new InstantCommand(() -> LimelightHelpers.setPipelineIndex("", 0))));

        m_eventMap.put("ConePickup",
            OneMechanism.runArmsSimultaneouslyCommand(ScoringPositions.AUTON_URPIGHT_CONE)
                .alongWith(new InstantCommand(() -> LimelightHelpers.setPipelineIndex("", 1))));

        m_eventMap.put("TippedConePickup",
            OneMechanism.runArmsSimultaneouslyCommand(ScoringPositions.ACQUIRE_FLOOR_CONE_TIPPED)
                .alongWith(new InstantCommand(() -> LimelightHelpers.setPipelineIndex("", 1))));

        m_eventMap.put("RunGripperIn", m_gripper.runMotorIn());
        m_eventMap.put("RunGripperOut", m_gripper.runMotorOut());
        m_eventMap.put("StopGripper", new InstantCommand(() -> m_gripper.beIdleMode()));

        m_eventMap.put("ArmRetract", OneMechanism.runArms(ScoringPositions.STOWED));

        m_eventMap.put("RunGripperSmart", m_gripper.runMotorIn().until(m_gripper.atCurrentThresholdSupplier()));

        m_eventMap.put("CubeTwo", new InstantCommand(() -> LimelightHelpers.setCropWindow("", CUBE_TWO_CROP[0],
            CUBE_TWO_CROP[1], CUBE_TWO_CROP[2], CUBE_TWO_CROP[3])));
        m_eventMap.put("CubeThree", new InstantCommand(() -> LimelightHelpers.setCropWindow("", CUBE_THREE_CROP[0],
            CUBE_THREE_CROP[1], CUBE_THREE_CROP[2], CUBE_THREE_CROP[3])));

        m_eventMap.put("FrontLocalize", new AddVisionMeasurement(drivetrain, m_frontAprilTagVision));
        m_eventMap.put("RearLocalize", new AddVisionMeasurement(drivetrain, m_rearAprilTagVision));

        m_eventMap.put("FrontLocalizeContinuous",
            new RepeatCommand(new AddVisionMeasurement(drivetrain, m_frontAprilTagVision)));
        m_eventMap.put("RearLocalizeContinuous",
            new RepeatCommand(new AddVisionMeasurement(drivetrain, m_rearAprilTagVision)));

        m_eventMap.put("FrontReset", new ResetPoseToVision(drivetrain, m_frontAprilTagVision));
        m_eventMap.put("RearReset", new ResetPoseToVision(drivetrain, m_rearAprilTagVision));
    }

    /**
     * Zero the arms based on the starting point of auton.
     * 
     * @return A {@link Command} that sets the arms to the offsets the arms live at
     *         at the beginning of a match.
     */
    public Command autonZero() {
        return new SequentialCommandGroup(new InstantCommand(() -> m_upperArm.setEncoderPosition(UPPER_ARM_OFFSET)),
            new InstantCommand(() -> m_lowerArm.setEncoderPosition(0.)));
    }

    /**
     * Sequence of commands to score the preloaded game piece.
     * 
     * @param mode
     *            Whether a cone or cube is preloaded.
     * @return A {@link Command} scoring the preloaded game piece.
     */
    public Command preloadScoreSequence(GamePieceMode mode) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> OneMechanism.setScoreMode(true)),
            new InstantCommand(() -> OneMechanism.setClimbMode(false)),
            mode == GamePieceMode.ORANGE_CONE ? OneMechanism.orangeModeCommand() : OneMechanism.purpleModeCommand(),

            autonZero(),
            
            new WaitCommand(0.1),
            new InstantCommand(() -> m_upperArm.runArmVbus(-0.3)),
            new WaitCommand(0.07),

            mode == GamePieceMode.ORANGE_CONE ? m_coneExtendCommand.get() : m_cubeExtendCommand.get(),
            m_gripper.modeSensitiveOutfeedCommand().withTimeout(0.4),

            m_stowCommand.get(),

            new InstantCommand(() -> OneMechanism.setScoreMode(false)));
    }

    /**
     * Cool preload score sequence
     * 
     * @return A {@link Command} to do our cool new trick
     */
    public Command coolPreloadScoreSequence() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> OneMechanism.setScoreMode(true)),
            new InstantCommand(() -> OneMechanism.setClimbMode(false)),
            OneMechanism.orangeModeCommand(),

            autonZero(),

            // wacky timing stuff
            new WaitCommand(0.02),
            new InstantCommand(() -> m_upperArm.runArmVbus(-0.5)),
            new WaitCommand(0.04),
            new InstantCommand(() -> m_lowerArm.runArmVbus(0.9)),
            new WaitCommand(0.04),
            new InstantCommand(() -> m_upperArm.runArmVbus(0.)),
            new WaitCommand(0.16),
            new InstantCommand(() -> m_lowerArm.runArmVbus(0.)),

            // new RunArmPID(ScoringPositions.STOWED.upperPosition, m_upperArm)
            // .alongWith(m_wrist.runToAngle(ScoringPositions.STOWED.wristAngle)).withTimeout(0.1),

            // // Run the lower arm down but idk
            // new WaitCommand(0.35).deadlineWith(new RunArmPID(10., m_lowerArm)),

            new WaitCommand(0.25).deadlineWith(
                new ParallelCommandGroup(
                    new RunArmPID(ScoringPositions.FLOOR_CUBE_SEEK.lowerPosition, m_lowerArm),
                    new RunArmPID(ScoringPositions.FLOOR_CUBE_SEEK.upperPosition, m_upperArm),
                    m_wrist.runToAngle(ScoringPositions.FLOOR_CUBE_SEEK.wristAngle))));
    }

    /**
     * Get the proper scoring sequence for the beginning of auton.
     * 
     * @param mode
     *            Whether a cube or cone is preloaded.
     * @param scoreHigh
     *            Whether to score the first piece high (true) or low (false)
     * 
     * @return A {@link Command} that runs the proper preload scoring sequence.
     */
    public Command getPreloadScoreSequence(GamePieceMode mode, boolean scoreHigh) {
        return scoreHigh ? preloadScoreSequence(mode) : coolPreloadScoreSequence();
    }

    /**
     * Load an auton path.
     * 
     * @param position
     *            What position this path is in (flat, bump, station)
     * @param part
     *            What part of the auton this is (acquire, score, balance, park)
     * @param pieceNum
     *            What piece this path is acquiring or scoring (for balance, the
     *            last piece scored)
     * @param scoreHigh
     *            Whether to score the preloaded game piece high or low.
     * @param data
     *            Any additional path data that may or may not be present.
     * 
     * @return The final trajectory.
     */
    public PathPlannerTrajectory loadPath(PathPosition position, PathPart part, String pieceNum, boolean scoreHigh,
        String data) {
        return Trajectories.loadPath(m_drivetrain, position, part, pieceNum, scoreHigh, data);
    }

    // ================================================
    // CHARGE STATION CUSTOM AUTOS
    // ================================================
    public BeakAutonCommand OnePieceMobilityBalance(boolean scoreHigh) {
        LinearFilter filter = LinearFilter.movingAverage(6);

        SnapDirection direction = scoreHigh ? SnapDirection.DOWN : SnapDirection.UP;
        Rotation2d initialRotation = scoreHigh ? Rotation2d.fromDegrees(180.) : new Rotation2d();

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, new Pose2d(1.73, 2.76, initialRotation),
            getPreloadScoreSequence(GamePieceMode.ORANGE_CONE, scoreHigh),

            // Drive fast until we hit the charge station
            new WaitUntilCommand(() -> scoreHigh ? m_drivetrain.getJerk() < -0.8 : m_drivetrain.getJerk() > 0.8)
                .deadlineWith(
                    new SnapToAngle(direction, () -> 2.5, () -> 0., () -> false, false, m_drivetrain)),

            // When the charge station first tips, drive until it's tipped the other way
            new WaitUntilCommand(() -> {
                double average = filter.calculate(m_drivetrain.getGyroPitchRotation2d().getDegrees());
                return scoreHigh ? average > 4 : average < -4;
            }).andThen(new WaitCommand(0.85))
                .deadlineWith(
                    new SnapToAngle(direction, () -> 1.25, () -> 0., () -> false, false, m_drivetrain)),

            // drive backwards and balance
            new WaitUntilCommand(() -> {
                double average = filter.calculate(m_drivetrain.getGyroPitchRotation2d().getDegrees());
                return scoreHigh ? average > 8 : average < -8;
            }).andThen(new WaitCommand(0.75)).deadlineWith(
                new SnapToAngle(direction, () -> -1.45, () -> 0., () -> false, false, m_drivetrain)),

            new QuadraticAutoBalance(m_drivetrain)
        //
        );

        return cmd;
    }

    // ================================================
    // ONE PIECE AUTONS
    // ================================================

    // SUBJECT TO REMOVAL
    public BeakAutonCommand OneBalance(PathPosition position, GamePieceMode preload) {
        PathPlannerTrajectory traj = loadPath(position, PathPart.Bal, "1", true, "");

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, traj.getInitialHolonomicPose(),
            position == PathPosition.Middle ? preloadScoreSequence(preload) : Commands.none(),
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
        BeakAutonCommand initialPath = Acquire(position, GamePieceMode.ORANGE_CONE, "2", true, true, "");

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, initialPath.getInitialPose(),
            initialPath,
            OneBalance(position, GamePieceMode.ORANGE_CONE)
        //
        );

        return cmd;
    }

    // ================================================
    // GENERIC AUTON PATHS
    // ================================================

    /**
     * Generic piece acquiring path.
     * 
     * @param position
     *            The position of this path.
     * @param mode
     *            Whether to switch to cone or cube mode before acquiring.
     * @param pieces
     *            The piece number of this path.
     * @param initialPath
     *            Whether or not this is the first path in the auton.
     * @param scoreHigh
     *            Whether to score the preloaded game piece high or low.
     * 
     * @return A @{@link BeakAutonCommand} to run the acquire path.
     * 
     * @see Trajectories
     */
    public BeakAutonCommand Acquire(PathPosition position, GamePieceMode mode, String pieces, boolean initialPath,
        boolean scoreHigh, String data) {
        PathPlannerTrajectory traj = loadPath(position, PathPart.Acquire, pieces, scoreHigh, data);

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, traj,
            initialPath ? getPreloadScoreSequence(GamePieceMode.ORANGE_CONE, scoreHigh) : Commands.none(),

            OneMechanism.getModeCommand(mode),
            m_drivetrain.getTrajectoryCommand(traj, m_eventMap)
        //
        );

        return cmd;
    }

    /**
     * Generic piece scoring path.
     * 
     * @param position
     *            The position of this path.
     * @param pieces
     *            The piece number of this path.
     * @param park
     *            Whether to park at the end or actually score the piece.
     * @param data
     *            Additional path data.
     * 
     * @return A {@link BeakAutonCommand} to run the score path.
     */
    public BeakAutonCommand Score(PathPosition position, String pieces, boolean park, String data) {
        PathPlannerTrajectory traj = loadPath(position, PathPart.Score, pieces, false, data);

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, traj,
            new InstantCommand(() -> OneMechanism.setScoreMode(true)),
            m_drivetrain.getTrajectoryCommand(traj, m_eventMap),

            // The arms start going to the high scoring position at the end of the path.
            // OneMechanism.runArms(ScoringPositions.SCORE_HIGH_CUBE).until(m_upperArmExtended),
            new WaitUntilCommand(m_upperArmExtended),
            park ? Commands.none() : m_gripper.runMotorOutSoft().withTimeout(0.4),

            m_stowCommand.get(),
            new InstantCommand(() -> OneMechanism.setScoreMode(false))
        //
        );

        return cmd;
    }

    /**
     * Run a charge station balance path.
     * 
     * @param position
     *            The position of this path.
     * @param pieces
     *            The piece number of this path.
     * 
     * @return A {@link BeakAutonCommand} to run the balance path.
     */
    public BeakAutonCommand Balance(PathPosition position, String pieces) {
        PathPlannerTrajectory traj = loadPath(position, PathPart.Bal, pieces, false, "");

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
    // AUTON SEQUENCES
    // ================================================

    /**
     * Two piece path.
     * 
     * @param position
     *            The position of this path.
     * @param balance
     *            Whether or not to run a balance sequence at the end of this path.
     * 
     * @return A {@link BeakAutonCommand} to run the two piece path.
     */
    public BeakAutonCommand TwoPiece(PathPosition position, boolean balance) {
        // Acquire and Score already have existing paths, so the full two piece is
        // simply a combination of the two.
        BeakAutonCommand initialPath = Acquire(position, GamePieceMode.PURPLE_CUBE, "2", true, true, "");

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, initialPath.getInitialPose(),
            initialPath,
            Score(position, "2", false, ""),
            balance ? Balance(position, "2") : Commands.none()
        //
        );

        return cmd;
    }

    // SUBJECT TO REMOVAL
    public BeakAutonCommand TwoQuarterPiecePark(PathPosition position) {
        PathPlannerTrajectory traj = loadPath(position, PathPart.Park, "2.25", false, "");

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, traj,
            m_drivetrain.getTrajectoryCommand(traj, m_eventMap) //
        );

        return cmd;
    }

    /**
     * Runs the two piece auton and parks near the next game piece.
     */
    public BeakAutonCommand TwoQuarterPiece(PathPosition position) {
        BeakAutonCommand initialPath = TwoPiece(position, false);

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, initialPath.getInitialPose(),
            initialPath,
            TwoQuarterPiecePark(position)
        //
        );

        return cmd;
    }

    /**
     * Three piece path.
     * 
     * @param position
     *            The position of this path.
     * @param balance
     *            Whether or not to run a balance sequence at the end of this path.
     * 
     * @return A {@link BeakAutonCommand} to run the two piece path.
     */
    public BeakAutonCommand ThreePiece(PathPosition position, boolean balance) {
        // The Three Piece paths are made to continue off of the two piece path. Rather
        // than doing everything again, we simply run the two piece auton and continue
        // where we left off for the three piece paths.
        BeakAutonCommand initialPath = TwoPiece(position, false);

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, initialPath.getInitialPose(),
            initialPath,
            Acquire(position, GamePieceMode.ORANGE_CONE, "3", false, false, ""),

            m_gripper.runMotorInWithoutReset().until(m_gripper.atCurrentThresholdSupplier())
                .until(m_gripper.hasGamePieceSupplier()).withTimeout(3.0),
            new WaitUntilCommand(m_gripper.hasGamePieceSupplier()),

            Score(position, "3", true, "")
        //
        );

        return cmd;
    }

    public BeakAutonCommand LimelightThreePiece(PathPosition position, GamePieceMode mode) {
        BeakAutonCommand initialPath = TwoPiece(position, false);

        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, initialPath.getInitialPose(),
            initialPath,
            Acquire(position, mode, "3", false, false, "Limelight"),
            new LimelightSquare(
                mode == GamePieceMode.ORANGE_CONE,
                false,
                () -> LimelightHelpers.getTY("") < 0. ? 4.5 + (1. / 12.) * (LimelightHelpers.getTY("")) : 4.5,
                () -> 0.0,
                m_drivetrain).withTimeout(0.66),
            Score(position, "3", true, "Limelight"));

        return cmd;
    }
}
