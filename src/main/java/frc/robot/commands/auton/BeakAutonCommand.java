// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.beaklib.drive.BeakDrivetrain;
import frc.robot.OneMechanism;
import frc.robot.Constants.FieldConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
/**
 * An auton path to run trajectories and other commands, storing the initial
 * pose of the path.
 */
public class BeakAutonCommand extends SequentialCommandGroup {
    private Pose2d m_initialPose;
    private PathPlannerTrajectory m_initialTrajectory;
    private BeakDrivetrain m_drivetrain;

    /** Default Constructor */
    public BeakAutonCommand() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
    }

    /**
     * Create a new BeakAutonCommand.
     * 
     * @param drivetrain
     *            The drivetrain to use for the paths.
     * @param initialTrajectory
     *            The first trajectory that is run by the robot.
     * @param commands
     *            A list of commands to run, in order. This is NOT limited to
     *            trajectory commands!
     */
    public BeakAutonCommand(BeakDrivetrain drivetrain, PathPlannerTrajectory initialTrajectory, Command... commands) {
        this(drivetrain, initialTrajectory.getInitialHolonomicPose(), commands);
    }

    /**
     * Create a new BeakAutonCommand.
     * 
     * @param drivetrain
     *            The drivetrain to use for the paths.
     * @param initialPose
     *            The initial pose of the robot.
     * @param commands
     *            A list of commands to run, in order. This is NOT limited to
     *            trajectory commands!
     */
    public BeakAutonCommand(BeakDrivetrain drivetrain, Pose2d initialPose, Command... commands) {
        m_drivetrain = drivetrain;
        setInitialPose(initialPose);

        // This is NECESSARY to be able to run autons several times per code deployment.
        for (Command command : commands) {
            CommandScheduler.getInstance().removeComposedCommand(command);
        }

        super.addCommands(commands);
        super.addRequirements(drivetrain);
    }

    /**
     * Resets the pose of the robot to the correct initial pose and runs the auton
     * path.
     * 
     * @return A {@link SequentialCommandGroup} that resets the robot pose and runs
     *         the path.
     */
    public SequentialCommandGroup resetPoseAndRun() {
        return new InstantCommand(() -> {
            Pose2d transformedPose = m_initialPose;

            // We need to invert the starting pose for the red alliance.
            if (DriverStation.getAlliance() == Alliance.Red) {
                Translation2d transformedTranslation = new Translation2d(m_initialPose.getX(),
                    FieldConstants.FIELD_WIDTH.getAsMeters() - m_initialPose.getY());
                Rotation2d transformedHeading = m_initialPose.getRotation().times(-1);

                transformedPose = new Pose2d(transformedTranslation, transformedHeading);
            }

            m_drivetrain.resetOdometry(transformedPose);
        }).andThen(this);
    }

    protected void setInitialPose(Pose2d initialPose) {
        m_initialPose = initialPose; // TODO: specify each drivetrain as holonomic or
                                     // not
    }

    /**
     * Get the initial pose of this auton command.
     * 
     * @return Initial pose of the path.
     */
    public Pose2d getInitialPose() {
        return m_initialPose;
    }
}
