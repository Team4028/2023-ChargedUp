// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.drive;

import java.util.Map;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.beaklib.drive.swerve.DrivetrainConfiguration;
import frc.lib.beaklib.gyro.BeakGyro;
import frc.lib.beaklib.motor.BeakMotorControllerGroup;
import frc.lib.beaklib.pid.BeakPIDConstants;
import frc.lib.beaklib.units.Velocity;

/** Base class for all differential (tank, kitbot, WCD) drivetrains. */
public class BeakDifferentialDrivetrain extends BeakDrivetrain {
    protected BeakMotorControllerGroup m_leftControllers;
    protected BeakMotorControllerGroup m_rightControllers;

    protected DifferentialDrivePoseEstimator m_odom;
    protected DifferentialDriveKinematics m_kinematics;

    protected DrivetrainConfiguration m_config;

    /**
     * Create a new Differential Drivetrain.
     * 
     * @param config
     *            {@link DrivetrainConfiguration} for this drivetrain
     * @param thetaPID
     *            PID Constants for autonomous rotation
     * @param drivePID
     *            PID Constants for autonomous driving
     * @param generatedDrivePID
     *            PID Constants for autonomous driving with generated paths
     * @param gyroInverted
     *            Whether or not the gyroscope is inverted.
     */
    public BeakDifferentialDrivetrain(
        DrivetrainConfiguration config,
        BeakPIDConstants thetaPID,
        BeakPIDConstants drivePID,
        BeakPIDConstants generatedDrivePID,
        boolean gyroInverted) {
        super(config.Physics, thetaPID, drivePID, generatedDrivePID, gyroInverted);

        m_config = config;

        m_kinematics = new DifferentialDriveKinematics(m_trackWidth.getAsMeters());
        m_odom = new DifferentialDrivePoseEstimator(m_kinematics, getGyroRotation2d(), 0., 0., new Pose2d());
    }

    /**
     * Setup motor and gyro configurations.
     * @param leftMotorControllers Motor Controllers that control the left side of the drivetrain.
     * @param rightMotorControllers Motor Controllers that control the right side of the drivetrain.
     * @param gyro The gyroscope in use for this drivetrain.
     */
    public void setup(
        BeakMotorControllerGroup leftMotorControllers,
        BeakMotorControllerGroup rightMotorControllers,
        BeakGyro gyro) {
        m_gyro = gyro;

        m_leftControllers = leftMotorControllers;
        m_rightControllers = rightMotorControllers;

        // Real-World Values
        m_leftControllers.setEncoderGearRatio(m_gearRatio);
        m_leftControllers.setWheelDiameter(m_wheelDiameter);

        m_rightControllers.setEncoderGearRatio(m_gearRatio);
        m_rightControllers.setWheelDiameter(m_wheelDiameter);

        // Current
        m_leftControllers.setSupplyCurrentLimit(m_config.DriveSupplyLimit);
        m_leftControllers.setStatorCurrentLimit(m_config.DriveStatorLimit);

        m_rightControllers.setSupplyCurrentLimit(m_config.DriveSupplyLimit);
        m_rightControllers.setStatorCurrentLimit(m_config.DriveStatorLimit);

        // PID
        m_leftControllers.setPID(m_config.DrivePID, 0);
        m_rightControllers.setPID(m_config.DrivePID, 0);
    }

    /* Differential-specific methods */

    /**
     * Open-loop drive with voltage input.
     * 
     * @param leftVolts
     *            Volts to send to the left side.
     * @param rightVolts
     *            Volts to send to the right side.
     */
    public void driveVolts(double leftVolts, double rightVolts) {
        m_leftControllers.setVoltage(leftVolts);
        m_rightControllers.setVoltage(rightVolts);
    }

    /**
     * Closed-loop drive with direct velocity input.
     * 
     * @param leftMetersPerSecond
     *            Left-side velocity.
     * @param rightMetersPerSecond
     *            Right-side velocity.
     */
    public void drive(double leftMetersPerSecond, double rightMetersPerSecond) {
        m_leftControllers.setVelocity(new Velocity(leftMetersPerSecond), m_feedForward.calculate(leftMetersPerSecond),
            0);
        m_rightControllers.setVelocity(new Velocity(rightMetersPerSecond),
            m_feedForward.calculate(rightMetersPerSecond),
            0);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            m_leftControllers.getSpeed().Value.getAsMetersPerSecond(),
            m_rightControllers.getSpeed().Value.getAsMetersPerSecond());
    }

    /* Overrides */
    @Override
    public void drive(double x, double y, double rot, boolean fieldRelative) {
        x *= m_maxVelocity.getAsMetersPerSecond();
        y *= m_maxVelocity.getAsMetersPerSecond();
        rot *= m_maxAngularVelocity.getAsRadiansPerSecond();

        ChassisSpeeds speeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, getRotation2d())
            : new ChassisSpeeds(x, y, rot);

        drive(speeds);
    }

    @Override
    public void drive(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);

        if (m_config.IsOpenLoop) {
            driveVolts(wheelSpeeds.leftMetersPerSecond / m_maxVelocity.getAsMetersPerSecond() * 12.,
                wheelSpeeds.rightMetersPerSecond / m_maxVelocity.getAsMetersPerSecond() * 12.);
        } else {
            drive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
        }
    }

    @Override
    public Command getTrajectoryCommand(PathPlannerTrajectory traj, Map<String, Command> eventMap) {
        Command pathFollowingCommand = m_config.IsOpenLoop ? new PPRamseteCommand(
            traj,
            this::getPoseMeters,
            new RamseteController(),
            m_feedForward,
            m_kinematics,
            this::getWheelSpeeds,
            createDriveController(),
            createDriveController(),
            this::driveVolts,
            this) :

            new PPRamseteCommand(
                traj,
                this::getPoseMeters,
                new RamseteController(),
                m_kinematics,
                this::drive,
                this);

        return new FollowPathWithEvents(
            pathFollowingCommand,
            traj.getMarkers(),
            eventMap);
    }

    @Override
    public Pose2d updateOdometry() {
        m_odom.updateWithTime(
            RobotController.getFPGATime() / 1000000.,
            getGyroRotation2d(),
            m_leftControllers.getDistance(true).Value.getAsMeters(),
            m_rightControllers.getDistance(true).Value.getAsMeters());

        return getPoseMeters();
    }

    @Override
    public void addVisionMeasurement(Pose2d estimatedPose, double timestamp) {
        Transform2d poseError = estimatedPose.minus(m_odom.getEstimatedPosition());

        if (!estimatedPose.equals(new Pose2d()) && !estimatedPose.equals(getPoseMeters()) &&
            Math.abs(poseError.getX()) < 0.5 &&
            Math.abs(poseError.getY()) < 0.5) {
            m_odom.addVisionMeasurement(estimatedPose, timestamp);
        }
    }

    @Override
    public Pose2d getPoseMeters() {
        return m_odom.getEstimatedPosition();
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        if (!pose.equals(new Pose2d()))
            m_odom.resetPosition(getGyroRotation2d(),
                m_leftControllers.getDistance(true).Value.getAsMeters(),
                m_rightControllers.getDistance(true).Value.getAsMeters(),
                pose);
    }

    @Override
    public boolean isHolonomic() {
        return false;
    }
}
