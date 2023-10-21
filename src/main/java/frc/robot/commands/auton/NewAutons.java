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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.beaklib.drive.swerve.BeakSwerveDrivetrain;
import frc.robot.OneMechanism;
import frc.robot.OneMechanism.GamePieceMode;
import frc.robot.OneMechanism.ScoringPositions;
import frc.robot.commands.arm.RunArmPID;
import frc.robot.commands.chassis.AddVisionMeasurement;
import frc.robot.commands.chassis.ResetPoseToVision;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.arms.LowerArm;
import frc.robot.subsystems.arms.UpperArm;
import frc.robot.subsystems.manipulator.Gripper;
import frc.robot.subsystems.manipulator.Wrist;
import frc.robot.utilities.LimelightHelpers;

/** Add your docs here. */
public class NewAutons {
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

    // TODO: attempt auton impl
    public enum NodeLocation {
        LOW(ScoringPositions.FLOOR_CUBE_SEEK, ScoringPositions.FLOOR_CUBE_SEEK, ScoringPositions.FLOOR_CUBE_SEEK), MID(
            ScoringPositions.SCORE_MID_CUBE, ScoringPositions.SCORE_MID_CONE, ScoringPositions.SCORE_MID_CUBE), HIGH(
                ScoringPositions.AUTON_PREP_CUBE, ScoringPositions.SCORE_HIGH_CONE, ScoringPositions.SCORE_HIGH_CUBE);

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

        m_cubeExtendCommand = () -> OneMechanism.runArms(ScoringPositions.SCORE_HIGH_CUBE);// .until(m_upperArmExtended);

        m_coneExtendCommand = () -> OneMechanism.runArms(ScoringPositions.SCORE_HIGH_CONE).until(m_upperArmExtended);

        // The event map is used for PathPlanner's FollowPathWithEvents function.
        // Almost all pickup, scoring, and localization logic is done through events.
        m_eventMap = new HashMap<String, Command>();

        m_eventMap.put("CubeHigh", m_cubeExtendCommand.get());
        m_eventMap.put("ConeHigh", m_coneExtendCommand.get());

        m_eventMap.put("CubeMid", OneMechanism.runArms(ScoringPositions.SCORE_MID_CUBE));
        m_eventMap.put("ConeMid", OneMechanism.runArms(ScoringPositions.SCORE_MID_CONE));

        m_eventMap.put("ScoreLow", OneMechanism.runArms(ScoringPositions.FLOOR_CUBE_SEEK));

        m_eventMap.put("CubePrep", OneMechanism.runArms(ScoringPositions.AUTON_PREP_CUBE));
        m_eventMap.put("ConePrep", OneMechanism.runArms(ScoringPositions.AUTON_PREP_CONE));

        // TODO: these should control LEDs.
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
        m_eventMap.put("KeepCube", OneMechanism.runArms(ScoringPositions.STOWED)
            .andThen(OneMechanism.runArms(ScoringPositions.KEEP_CUBE_DISABLE)));

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

        AutoBuilder.configureHolonomic(
            () -> m_drivetrain.getPoseMeters(),
            m_drivetrain::resetOdometry, 
            m_drivetrain::getChassisSpeeds,
            m_drivetrain::drive,
            // TODO: locations
            new HolonomicPathFollowerConfig(m_drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond(), 1, new ReplanningConfig()),
            m_drivetrain);
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
            new InstantCommand(() -> OneMechanism.buckeyeConstants()),

            autonZero(),

            new WaitCommand(0.1),
            new InstantCommand(() -> m_upperArm.runArmVbus(-0.3)),
            new WaitCommand(0.07),

            OneMechanism.runArms(ScoringPositions.AUTON_PRELOAD_SCORE),//.until(m_upperArmExtended),
            m_gripper.modeSensitiveOutfeedCommand().withTimeout(0.4),
            new InstantCommand(() -> OneMechanism.buckeyeConstants()),

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
    
    public PathPlannerAuto loadAuto(String name) {
        return new PathPlannerAuto(name);
    }
}
