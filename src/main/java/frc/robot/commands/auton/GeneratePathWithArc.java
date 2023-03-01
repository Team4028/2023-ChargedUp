// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.beaklib.drive.BeakDrivetrain;
import frc.lib.beaklib.units.Velocity;

// credit: https://github.com/HaMosad1657/MiniProject2023/blob/chassis/src/main/java/frc/robot/commands/drivetrain/FollowGeneratedTrajectoryCommand.java
public class GeneratePathWithArc extends CommandBase {
    private PIDController m_xController, m_yController, m_thetaController;

    private PPHolonomicDriveController m_driveController;

    private Timer m_timer;

    private PathPlannerTrajectory m_traj;

    private Supplier<Pose2d> m_poseSupplier;
    private Pose2d m_desiredPose;

    private Pose2d m_currentPose;

    private PathPlannerState m_setpoint;

    private Pose2d m_positionTolerance;

    private BeakDrivetrain m_drivetrain;

    Field2d field = new Field2d();

    /** Creates a new GeneratePath. */
    public GeneratePathWithArc(Supplier<Pose2d> desiredPose, BeakDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        m_poseSupplier = desiredPose;

        m_positionTolerance = new Pose2d(
            0.0254, // 1 inch
            0.0254,
            Rotation2d.fromDegrees(0.5));

        m_timer = new Timer();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_desiredPose = m_poseSupplier.get();

        if (m_desiredPose.equals(m_drivetrain.getPoseMeters()) || m_desiredPose.equals(new Pose2d())) {
            this.cancel();
        }

        // Set up PID controllers
        m_xController = m_drivetrain.createGeneratedDriveController();
        m_yController = m_drivetrain.createGeneratedDriveController();
        m_thetaController = m_drivetrain.createAutonThetaController();

        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // The drive controller takes in three PID controllers (x, y, theta)
        m_driveController = new PPHolonomicDriveController(
            m_xController,
            m_yController,
            m_thetaController);

        // Note: we also have to enable the controller
        m_driveController.setTolerance(m_positionTolerance);
        m_driveController.setEnabled(true);

        // Generate a trajectory and start the timer
        m_traj = generateTrajectoryToPose(m_desiredPose);

        Logger.getInstance().recordOutput("Desired Pose", m_desiredPose);

        field.setRobotPose(m_desiredPose);
        SmartDashboard.putData("Node Pose", field);

        m_timer.reset();
        m_timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Gets the setpoint--i.e. the next target position. This is used
        // by the drive controller to determine "where" it should be
        // on the next cycle.
        if (m_traj.getStates().size() > 0) {
            m_setpoint = (PathPlannerState) m_traj.sample(m_timer.get() + 0.02);

            // Gets the current pose
            m_currentPose = m_drivetrain.getPoseMeters();

            // The drive controller's calculation takes in the current position
            // and the target position, and outputs a ChassisSpeeds object.
            // This is then passed into the drivetrain's drive method.
            m_drivetrain.drive(
                m_driveController.calculate(
                    m_currentPose,
                    m_setpoint));
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        m_timer.reset();
        m_driveController.setEnabled(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Ends when it's at the target

        Transform2d error = m_desiredPose.minus(m_drivetrain.getPoseMeters()); // TODO: why
        Translation2d translationError = error.getTranslation();
        Rotation2d rotationError = error.getRotation();

        Translation2d translationTolerance = m_positionTolerance.getTranslation();
        Rotation2d rotationTolerance = m_positionTolerance.getRotation();

        // return (m_driveController.atReference());
        // TODO: bruh

        return Math.abs(translationError.getX()) < translationTolerance.getX()
            && Math.abs(translationError.getY()) < translationTolerance.getY()
            && Math.abs(rotationError.getRadians()) < rotationTolerance.getRadians();
    }

    /**
     * Custom trajectory generation thing to add an arc.
     * 
     * @return {@link PathPlannerTrajectory} that leads the robot to the desired
     *         pose.
     */
    public PathPlannerTrajectory generateTrajectoryToPose(Pose2d desiredPose) {
        Pose2d robotPose = m_drivetrain.getPoseMeters();

        // If the current position is equal to the target position (for example, no
        // target is visible if using vision pose), return a blank trajectory that does
        // nothing. This saves computational time and is generally good practice.
        if (robotPose.equals(desiredPose)) {
            return new PathPlannerTrajectory();
        }

        double drivetrainMaxVelocity = m_drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * 0.25;

        // PathPlanner takes in these constraints to determine maximum speed and
        // acceleration. In order to maximize precision, we go at quarter speed.
        PathConstraints constraints = new PathConstraints(
            drivetrainMaxVelocity,
            drivetrainMaxVelocity);

        // Add our path points--start at the current robot pose and end at the desired
        // pose.
        List<PathPoint> points = new ArrayList<PathPoint>();

        // This is essentially fancy logic to ensure the heading (direction of travel)
        // is correct.
        Translation2d midpointTranslation = desiredPose.getTranslation().plus(robotPose.getTranslation()).div(2)
            .plus(new Translation2d(0.15, 0.));

        // The heading should be the angle between the desired point and the current
        // point.
        Rotation2d midpointHeading = new Rotation2d(Math.atan2(
            desiredPose.getTranslation().getY() - midpointTranslation.getY(),
            desiredPose.getTranslation().getX() - midpointTranslation.getX()));

        PathPoint startPoint = new PathPoint(
            robotPose.getTranslation(),
            new Rotation2d(Math.atan2(
                midpointTranslation.getY() - robotPose.getTranslation().getY(),
                midpointTranslation.getX() - robotPose.getTranslation().getX())),
            robotPose.getRotation());

        PathPoint midpoint = new PathPoint(
            midpointTranslation,
            midpointHeading,
            desiredPose.getRotation(),
            drivetrainMaxVelocity);

        PathPoint endPoint = new PathPoint(
            desiredPose.getTranslation(),
            new Rotation2d(Math.PI).minus(midpointHeading), // The end heading should be the opposite of the previous
                                                             // heading
            desiredPose.getRotation());

        points.add(startPoint);
        points.add(midpoint);
        points.add(endPoint);

        points.add(new PathPoint(desiredPose.getTranslation(), desiredPose.getRotation(), desiredPose.getRotation()));

        // PathPlanner has a built in path generation function!
        PathPlannerTrajectory traj = PathPlanner.generatePath(constraints, points);

        return traj;
    }
}
