// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.beaklib.drive.swerve.BeakSwerveDrivetrain;
import frc.robot.OneMechanism;
import frc.robot.OneMechanism.GamePieceMode;
import frc.robot.OneMechanism.ScoringPositions;
import frc.robot.commands.arm.RunArmPID;
import frc.robot.commands.chassis.QuadraticAutoBalance;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.arms.LowerArm;
import frc.robot.subsystems.arms.UpperArm;
import frc.robot.subsystems.manipulator.Gripper;
import frc.robot.subsystems.manipulator.Wrist;
import frc.robot.utilities.Trajectories.PathPosition;

/** Add your docs here. */
public class NewAutons {
    // Auton Constants
    private static final double UPPER_ARM_OFFSET = 18.4;
    // private static final double LOWER_ARM_OFFSET = 0.86;

    // private static final double[] CUBE_TWO_CROP = { -1, 0.57, -1, 0.81 };
    // private static final double[] CUBE_THREE_CROP = { -0.57, 1, -1, 0.81 };

    // Global Subsystems -- initialized in the constructor
    private final BeakSwerveDrivetrain m_drivetrain;

    private final LowerArm m_lowerArm;
    private final UpperArm m_upperArm;
    private final Wrist m_wrist;
    private final Gripper m_gripper;

    private final Vision m_frontAprilTagVision;
    private final Vision m_rearAprilTagVision;

    private final BooleanSupplier m_upperArmStowed;
    private final BooleanSupplier m_upperArmExtended;

    private final Supplier<Command> m_stowCommand;

    private FloorPickup m_nextFloorPickup;
    private NodeLocation m_nextNodeLocation;

    // TODO: attempt auton impl
    public enum NodeLocation {
        LOW(ScoringPositions.FLOOR_CUBE_SEEK, ScoringPositions.FLOOR_CUBE_SEEK, ScoringPositions.FLOOR_CUBE_SEEK), //
        MID(ScoringPositions.SCORE_MID_CUBE, ScoringPositions.SCORE_MID_CONE, ScoringPositions.SCORE_MID_CUBE), //
        HIGH(ScoringPositions.AUTON_PREP_CUBE, ScoringPositions.SCORE_HIGH_CONE, ScoringPositions.SCORE_HIGH_CUBE);

        /** Scoring location for arm prep. */
        public ScoringPositions PrepPosition;

        /** Scoring location for cone scoring. */
        public ScoringPositions ConePosition;

        private NodeLocation(ScoringPositions prepPosition, ScoringPositions conePosition,
                ScoringPositions cubePosition) {
            PrepPosition = prepPosition;
            ConePosition = conePosition;
            CubePosition = cubePosition;
        }

        /** Scoring location for cube scoring. */
        public ScoringPositions CubePosition;
    }

    public enum FloorPickup {
        CUBE(ScoringPositions.ACQUIRE_FLOOR_CUBE, GamePieceMode.PURPLE_CUBE),
        TIPPED_CONE(ScoringPositions.ACQUIRE_FLOOR_CONE_TIPPED, GamePieceMode.ORANGE_CONE),
        UPRIGHT_CONE(ScoringPositions.ACQUIRE_FLOOR_CONE_UPRIGHT, GamePieceMode.ORANGE_CONE);

        /** Location for pickup of this piece. */
        public ScoringPositions PickupPosition;

        /** Mode for this piece. */
        public GamePieceMode Mode;

        private FloorPickup(ScoringPositions pickupPosition, GamePieceMode mode) {
            PickupPosition = pickupPosition;
            Mode = mode;
        }
    }

    // Subsystem & Event setup
    public NewAutons(
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
        m_upperArmExtended = () -> (m_upperArm.getError() <= 1.25)
                && (m_lowerArm.getError() <= 0.6 * m_lowerArm.getDistanceToTravel());

        m_stowCommand = () -> OneMechanism.runArms(ScoringPositions.STOWED).until(m_upperArmStowed);

        
        // m_eventMap.put("RunGripperSmart",
        // m_gripper.runMotorIn().until(m_gripper.atCurrentThresholdSupplier()));

        // m_eventMap.put("FrontLocalize", new AddVisionMeasurement(drivetrain,
        // m_frontAprilTagVision));
        // m_eventMap.put("RearLocalize", new AddVisionMeasurement(drivetrain,
        // m_rearAprilTagVision));

        // m_eventMap.put("FrontLocalizeContinuous",
        // new RepeatCommand(new AddVisionMeasurement(drivetrain,
        // m_frontAprilTagVision)));
        // m_eventMap.put("RearLocalizeContinuous",
        // new RepeatCommand(new AddVisionMeasurement(drivetrain,
        // m_rearAprilTagVision)));

        // m_eventMap.put("FrontReset", new ResetPoseToVision(drivetrain,
        // m_frontAprilTagVision));
        // m_eventMap.put("RearReset", new ResetPoseToVision(drivetrain,
        // m_rearAprilTagVision));

        AutoBuilder.configureHolonomic(
                () -> m_drivetrain.getPoseMeters(),
                m_drivetrain::resetOdometry,
                m_drivetrain::getChassisSpeeds,
                m_drivetrain::drive,
                // TODO: locations
                new HolonomicPathFollowerConfig(m_drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond(), 1,
                        new ReplanningConfig()),
                m_drivetrain);
    }

    Map<FloorPickup, Command> createInfeedMap() {
        Map<FloorPickup, Command> map = new HashMap<>();

        for (FloorPickup p : FloorPickup.values()) {
            map.put(p, OneMechanism.runArmsSimultaneouslyCommand(p.PickupPosition));
        }

        return map;
    }

    Map<NodeLocation, Command> createPrepMap() {
        Map<NodeLocation, Command> map = new HashMap<>();

        for (NodeLocation n : NodeLocation.values()) {
            map.put(n, OneMechanism.runArms(n.PrepPosition));
        }

        return map;
    }

    Map<NodeLocation, Command> createScoreMap() {
        Map<NodeLocation, Command> map = new HashMap<>();

        for (NodeLocation n : NodeLocation.values()) {
            map.put(n, new ConditionalCommand(
                    OneMechanism.runArms(n.CubePosition),
                    OneMechanism.runArms(n.ConePosition),
                    () -> m_nextFloorPickup == FloorPickup.CUBE));
        }

        return map;
    }

    void registerCommands() {
        NamedCommands.registerCommand("Retract",
                OneMechanism.runArms(ScoringPositions.STOWED).alongWith(m_gripper.idleModeCommand())
                        .until(m_upperArmStowed).alongWith(OneMechanism.setScoreModeCommand(false)));

        NamedCommands.registerCommand("Outfeed", m_gripper.modeSensitiveOutfeedCommand());
        NamedCommands.registerCommand("Infeed", m_gripper.runMotorIn().alongWith(
                new MultiConditionalCommand<FloorPickup>(createInfeedMap(), () -> m_nextFloorPickup, m_gripper)));

        NamedCommands.registerCommand("Prep", new MultiConditionalCommand<>(createPrepMap(), () -> m_nextNodeLocation,
                m_lowerArm, m_upperArm, m_wrist));
        NamedCommands.registerCommand("Score", new MultiConditionalCommand<>(createScoreMap(), () -> m_nextNodeLocation,
                m_lowerArm, m_upperArm, m_wrist).until(m_upperArmExtended)
                .andThen(m_gripper.modeSensitiveOutfeedCommand()).alongWith(OneMechanism.setScoreModeCommand(true)));

        NamedCommands.registerCommand("SetLEDs", OneMechanism.runtimeModeCommand(() -> m_nextFloorPickup.Mode));
        NamedCommands.registerCommand("Climb", OneMechanism.setClimbModeCommand(true));

        NamedCommands.registerCommand("Balance", new QuadraticAutoBalance(m_drivetrain));
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
     *             Whether a cone or cube is preloaded.
     * @return A {@link Command} scoring the preloaded game piece.
     */
    public Command preloadScoreSequence(GamePieceMode mode) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> OneMechanism.setScoreMode(true)),
                new InstantCommand(() -> OneMechanism.setClimbMode(false)),
                OneMechanism.getModeCommand(mode),
                new InstantCommand(() -> OneMechanism.buckeyeConstants()),

                autonZero(),

                new WaitCommand(0.1),
                new InstantCommand(() -> m_upperArm.runArmVbus(-0.3)),
                new WaitCommand(0.07),

                OneMechanism.runArms(ScoringPositions.AUTON_PRELOAD_SCORE), // .until(m_upperArmExtended),
                m_gripper.modeSensitiveOutfeedCommand().withTimeout(0.4),
                new InstantCommand(() -> OneMechanism.worldsConstants()),

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
     *                  Whether a cube or cone is preloaded.
     * @param scoreHigh
     *                  Whether to score the first piece high (true) or low (false)
     * 
     * @return A {@link Command} that runs the proper preload scoring sequence.
     */
    public Command getPreloadScoreSequence(GamePieceMode mode, boolean scoreHigh) {
        return scoreHigh ? preloadScoreSequence(mode) : coolPreloadScoreSequence();
    }

    public PathPlannerAuto loadAuto(String name) {
        return new PathPlannerAuto(name);
    }

    public PathPlannerAuto Two(PathPosition position, boolean balance) {
        return loadAuto("2 " + position.name() + (balance ? " Balance" : ""));
    }

    public PathPlannerAuto Three(PathPosition position, boolean balance) {
        return loadAuto("3 " + position.name() + (balance ? " Balance" : ""));
    }
}
