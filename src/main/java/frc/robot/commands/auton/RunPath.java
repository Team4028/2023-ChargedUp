// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utilities.drive.BeakDrivetrain;

// credit: https://github.com/HaMosad1657/MiniProject2023/blob/chassis/src/main/java/frc/robot/commands/drivetrain/FollowGeneratedTrajectoryCommand.java
public class RunPath extends CommandBase {
    private PIDController m_xController, m_yController, m_thetaController;

    private PPHolonomicDriveController m_driveController;

    private Timer m_timer;

    private final PathPlannerTrajectory m_traj;

    private Pose2d m_currentPose;

    private PathPlannerState m_setpoint;

    private Pose2d m_positionTolerance;

    private BeakDrivetrain m_drivetrain;

    /** Creates a new RunPath. */
    public RunPath(PathPlannerTrajectory traj, BeakDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        m_traj = traj;

        m_drivetrain.resetOdometry(traj.getInitialHolonomicPose());

        m_positionTolerance = new Pose2d(
                0.1, // 4 inches
                0.1,
                Rotation2d.fromDegrees(2.0));
        
        m_timer = new Timer();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Set up PID controllers
        m_xController = m_drivetrain.createDriveController();
        m_yController = m_drivetrain.createDriveController();
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

        m_timer.reset();
        m_timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Gets the setpoint--i.e. the next target position. This is used
        // by the drive controller to determine "where" it should be
        // on the next cycle.
        m_setpoint = (PathPlannerState) m_traj.sample(m_timer.get() + 0.02);

        SmartDashboard.putNumber("Trajectory Pose X", m_setpoint.poseMeters.getX());
        SmartDashboard.putNumber("Trajectory Pose Y", m_setpoint.poseMeters.getX());
        SmartDashboard.putNumber("Trajectory Pose Rotation", m_setpoint.holonomicRotation.getDegrees());

        // Gets the current pose
        m_currentPose = m_drivetrain.getPoseMeters();

        // The drive controller's calculation takes in the current position
        // and the target position, and outputs a ChassisSpeeds object.
        // This is then passed into the drivetrain's drive method.

        ChassisSpeeds speeds = m_driveController.calculate(
            m_currentPose,
            m_setpoint);
        
        SmartDashboard.putNumber("X output", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("y output", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("theta output", speeds.omegaRadiansPerSecond);
        m_drivetrain.drive(
                m_driveController.calculate(
                        m_currentPose,
                        m_setpoint));
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
        // Ends when it's at the target while also not ending "too early"
        return (m_traj.getTotalTimeSeconds() < m_timer.get() && m_driveController.atReference());
    }
}
