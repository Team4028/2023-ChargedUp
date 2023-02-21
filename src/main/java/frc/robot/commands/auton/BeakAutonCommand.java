// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.utilities.drive.BeakDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BeakAutonCommand extends SequentialCommandGroup {
    /** Creates a new BeakAutonCommand. */
    private Pose2d initialPose;
    private BeakDrivetrain m_drivetrain;

    public BeakAutonCommand() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
    }

    public BeakAutonCommand(BeakDrivetrain drivetrain, PathPlannerTrajectory initialTrajectory, Command... commands) {
        this(drivetrain, initialTrajectory.getInitialHolonomicPose(), commands);
    }

    public BeakAutonCommand(BeakDrivetrain drivetrain, Pose2d initialPose, Command... commands) {
        m_drivetrain = drivetrain;
        setInitialPose(initialPose);

        super.addCommands(commands);
        super.addRequirements(drivetrain);
    }

    public SequentialCommandGroup resetPoseAndRun() {
        return new InstantCommand(() -> m_drivetrain.resetOdometry(initialPose)).andThen(this);
    }

    protected void setInitialPose(Pose2d initialPose) {
        this.initialPose = initialPose; // TODO: specify each drivetrain as holonomic or
                                                                        // not
    }

    public Pose2d getInitialPose() {
        return initialPose;
    }
}
