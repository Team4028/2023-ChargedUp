// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utilities.drive.BeakDrivetrain;
import frc.robot.utilities.drive.BeakHolonomicDriveController;

// credit: https://github.com/HaMosad1657/MiniProject2023/blob/chassis/src/main/java/frc/robot/commands/drivetrain/FollowGeneratedTrajectoryCommand.java
public class NewGeneratePath extends CommandBase {
    private PIDController m_xController, m_yController, m_thetaController;
    private PIDController m_generatedXController, m_generatedYController, m_generatedThetaController;

    private BeakHolonomicDriveController m_driveController;

    private final Timer m_timer;
    private double m_timerOffset;

    private PathPlannerTrajectory m_traj;
    private final PathPlannerTrajectory m_plannedTraj;

    private List<EventMarker> m_markers;

    private final Supplier<Pose2d> m_poseSupplier;
    private Pose2d m_desiredPose;

    private Pose2d m_currentPose;

    private PathPlannerState m_setpoint;
    private PathPlannerState m_plannedSetpoint;

    private final Pose2d m_positionTolerance;

    private final BeakDrivetrain m_drivetrain;

    /** Creates a new NewGeneratePath. */
    public NewGeneratePath(PathPlannerTrajectory plannedTraj, Supplier<Pose2d> desiredPose, BeakDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        m_poseSupplier = desiredPose;
        m_plannedTraj = plannedTraj;

        m_positionTolerance = new Pose2d(
                0.1, // 4 inches
                0.1,
                Rotation2d.fromDegrees(0.8));

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

        m_generatedXController = m_drivetrain.createGeneratedDriveController();
        m_generatedYController = m_drivetrain.createGeneratedDriveController();
        m_generatedThetaController = m_drivetrain.createAutonThetaController();

        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // The drive controller takes in three PID controllers (x, y, theta)
        m_driveController = new BeakHolonomicDriveController(
                m_xController,
                m_yController,
                m_thetaController,
                m_generatedXController,
                m_generatedYController,
                m_generatedThetaController);

        // Note: we also have to enable the controller
        m_driveController.setTolerance(m_positionTolerance);
        m_driveController.setEnabled(true);

        // start timer
        m_timer.reset();
        m_timer.start();

        m_markers = new ArrayList<EventMarker>(m_plannedTraj.getMarkers());

        m_plannedSetpoint = null;// new PathPlannerState();
        m_traj = null;
        m_desiredPose = null;

        SmartDashboard.putBoolean("useVsiion?", false);
        SmartDashboard.putBoolean("aint no way", false);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Gets the setpoint--i.e. the next target position. This is used
        // by the drive controller to determine "where" it should be
        // on the next cycle.
        m_setpoint = (PathPlannerState) m_plannedTraj.sample(m_timer.get() + 0.02);

        // Gets the current pose
        m_currentPose = m_drivetrain.getPoseMeters();

        // System.err.println(m_markers.size() > 0);
        // System.out.println(m_timer.get() >= m_markers.get(0).timeSeconds);
        SmartDashboard.putNumber("Current Pose", m_timer.get());

        if (m_markers.size() > 0) {
            SmartDashboard.putNumber("Marker pose", m_markers.get(0).timeSeconds);
        }

        if (m_markers.size() > 0
                && m_timer.get() > m_markers.get(0).timeSeconds) {
            // && m_currentPose.getTranslation().getNorm() >
            // m_markers.get(0).positionMeters.getNorm()) {
            PathPlannerTrajectory.EventMarker marker = m_markers.remove(0);

            for (String name : marker.names) {
                System.out.println(name);
                if (name.equals("useVision")) {
                    SmartDashboard.putBoolean("useVsiion?", true);
                    m_desiredPose = m_poseSupplier.get();
                }
            }
        }

        if ((m_desiredPose != null && m_desiredPose != m_currentPose)
                && (m_traj == null)) {
            m_traj = m_drivetrain.generateTrajectoryToPose(m_desiredPose);
        }

        SmartDashboard.putBoolean("Trajectory bruh", m_plannedSetpoint != null);
        SmartDashboard.putBoolean("Trajectory exist?", m_traj != null && m_traj.getStates().size() > 0);

        if (m_traj != null && m_traj.getStates().size() > 0) {
            m_timerOffset = m_timer.get();
            m_plannedSetpoint = (PathPlannerState) m_traj.sample(m_timer.get() - m_timerOffset + 0.02);
        }

        // The drive controller's calculation takes in the current position
        // and the target position, and outputs a ChassisSpeeds object.
        // This is then passed into the drivetrain's drive method.
        ChassisSpeeds output = m_driveController.calculate(
                m_currentPose,
                m_setpoint,
                m_plannedSetpoint);

        SmartDashboard.putNumber("vxMetersPerSecond", output.vxMetersPerSecond);
        SmartDashboard.putNumber("vyMetersPerSecond", output.vyMetersPerSecond);
        SmartDashboard.putNumber("omegaRadiansPerSecond", output.omegaRadiansPerSecond);

        m_drivetrain.drive(
                output);
        SmartDashboard.putNumber("Weight", m_driveController.getWeight());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        m_timer.reset();
        m_driveController.setEnabled(false);

        SmartDashboard.putBoolean("aint no way", true);

        if (m_desiredPose != null) {
            Transform2d poseDiff = m_drivetrain.getPoseMeters().minus(m_desiredPose);
            SmartDashboard.putNumber("X error", poseDiff.getX());
            SmartDashboard.putNumber("Y error", poseDiff.getY());
            SmartDashboard.putNumber("Theta error", poseDiff.getRotation().getDegrees());

            Field2d field = new Field2d();
            field.setRobotPose(m_desiredPose);
            SmartDashboard.putData("Vision Desired Pose", field);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Ends when it's at the target while also not ending "too early"
        if (m_traj != null)
            return (m_traj.getTotalTimeSeconds() < m_timer.get() - 0.5 - m_timerOffset &&
            m_driveController.atReference());
            // return false;
        else
            return (m_plannedTraj.getTotalTimeSeconds() < m_timer.get() - 0.5 &&
            m_driveController.atReference());
            // return false;
    }
}
