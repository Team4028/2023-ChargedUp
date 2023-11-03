// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.commands.PathfindLTV;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.beaklib.drive.swerve.DrivetrainConfiguration;
import frc.lib.beaklib.gyro.BeakGyro;
import frc.lib.beaklib.motor.BeakMotorControllerGroup;
import frc.lib.beaklib.pid.BeakPIDConstants;

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
     *                          {@link DrivetrainConfiguration} for this drivetrain
     * @param thetaPID
     *                          PID Constants for autonomous rotation
     * @param drivePID
     *                          PID Constants for autonomous driving
     * @param generatedDrivePID
     *                          PID Constants for autonomous driving with generated
     *                          paths
     * @param gyroInverted
     *                          Whether or not the gyroscope is inverted.
     */
    public BeakDifferentialDrivetrain(
            DrivetrainConfiguration config,
            BeakPIDConstants thetaPID,
            BeakPIDConstants drivePID,
            boolean gyroInverted) {
        super(config.Physics, thetaPID, drivePID, gyroInverted);

        m_config = config;

        m_kinematics = new DifferentialDriveKinematics(Physics.TrackWidth.in(Meters));
        m_odom = new DifferentialDrivePoseEstimator(m_kinematics, getGyroRotation2d(), 0., 0., new Pose2d());
    }

    /**
     * Setup motor and gyro configurations.
     * 
     * @param leftMotorControllers  Motor Controllers that control the left side of
     *                              the drivetrain.
     * @param rightMotorControllers Motor Controllers that control the right side of
     *                              the drivetrain.
     * @param gyro                  The gyroscope in use for this drivetrain.
     */
    public void setup(
            BeakMotorControllerGroup leftMotorControllers,
            BeakMotorControllerGroup rightMotorControllers,
            BeakGyro gyro) {
        m_gyro = gyro;

        m_leftControllers = leftMotorControllers;
        m_rightControllers = rightMotorControllers;

        // Real-World Values
        m_leftControllers.setEncoderGearRatio(Physics.DriveGearRatio);
        m_leftControllers.setWheelDiameter(Physics.WheelDiameter);

        m_rightControllers.setEncoderGearRatio(Physics.DriveGearRatio);
        m_rightControllers.setWheelDiameter(Physics.WheelDiameter);

        // Current
        m_leftControllers.setSupplyCurrentLimit(m_config.DriveSupplyLimit);
        m_leftControllers.setStatorCurrentLimit(m_config.DriveStatorLimit);

        m_rightControllers.setSupplyCurrentLimit(m_config.DriveSupplyLimit);
        m_rightControllers.setStatorCurrentLimit(m_config.DriveStatorLimit);

        // PID
        m_leftControllers.setSlot(0);
        m_rightControllers.setSlot(0);

        m_leftControllers.setPID(m_config.DrivePID);
        m_rightControllers.setPID(m_config.DrivePID);
    }

    /* Differential-specific methods */

    /**
     * Open-loop drive with voltage input.
     * 
     * @param leftVolts
     *                   Volts to send to the left side.
     * @param rightVolts
     *                   Volts to send to the right side.
     */
    public void driveVolts(double leftVolts, double rightVolts) {
        m_leftControllers.setVoltage(leftVolts);
        m_rightControllers.setVoltage(rightVolts);
    }

    /**
     * Closed-loop drive with direct velocity input.
     * 
     * @param leftVelocity
     *                      Left-side velocity.
     * @param rightVelocity
     *                      Right-side velocity.
     */
    public void drive(Measure<Velocity<Distance>> leftVelocity, Measure<Velocity<Distance>> rightVelocity) {
        double arbFeedforward = Physics.Feedforward.calculate(leftVelocity.in(MetersPerSecond));

        m_leftControllers.setNextArbFeedforward(arbFeedforward);
        m_rightControllers.setNextArbFeedforward(arbFeedforward);

        m_leftControllers.setVelocity(leftVelocity);
        m_rightControllers.setVelocity(rightVelocity);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                m_leftControllers.getSpeed().Value.in(MetersPerSecond),
                m_rightControllers.getSpeed().Value.in(MetersPerSecond));
    }

    /* Overrides */
    @Override
    public void drive(double x, double y, double rot, boolean fieldRelative) {
        x *= Physics.MaxVelocity.in(MetersPerSecond);
        y *= Physics.MaxVelocity.in(MetersPerSecond);
        rot *= Physics.MaxAngularVelocity.in(RadiansPerSecond);

        ChassisSpeeds speeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, getRotation2d())
                : new ChassisSpeeds(x, y, rot);

        drive(speeds);
    }

    @Override
    public void drive(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);

        if (m_config.IsOpenLoop) {
            driveVolts(wheelSpeeds.leftMetersPerSecond / Physics.MaxVelocity.in(MetersPerSecond) * 12.,
                    wheelSpeeds.rightMetersPerSecond / Physics.MaxVelocity.in(MetersPerSecond) * 12.);
        } else {
            drive(MetersPerSecond.of(wheelSpeeds.leftMetersPerSecond),
                    MetersPerSecond.of(wheelSpeeds.rightMetersPerSecond));
        }
    }

    @Override
    public PathfindingCommand pathFindingCommand(Pose2d desiredPose, double scale) {
        // todo: scale
        return new PathfindLTV(desiredPose.getTranslation(),
                new PathConstraints(1, 1, 1, 1),
                this::getPoseMeters,
                this::getChassisSpeeds,
                this::drive,
                0.02);
    }

    @Override
    public Pose2d updateOdometry() {
        m_odom.updateWithTime(
                RobotController.getFPGATime() / 1000000.,
                getGyroRotation2d(),
                m_leftControllers.getDistance(true).Value.in(Meters),
                m_rightControllers.getDistance(true).Value.in(Meters));

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
                    m_leftControllers.getDistance(true).Value.in(Meters),
                    m_rightControllers.getDistance(true).Value.in(Meters),
                    pose);
    }

    @Override
    public boolean isHolonomic() {
        return false;
    }

    @Override
    public ChassisSpeeds getChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getWheelSpeeds());
    }
}
