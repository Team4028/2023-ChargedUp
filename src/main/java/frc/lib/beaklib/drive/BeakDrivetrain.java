// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.drive;

import java.util.Map;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.beaklib.pid.BeakPIDConstants;
import frc.lib.beaklib.subsystem.BeakGyroSubsystem;

/** Base drivetrain class. */
public class BeakDrivetrain extends BeakGyroSubsystem {
    protected Pose2d m_pose;

    public RobotPhysics Physics;

    protected ProfiledPIDController m_thetaController;
    protected BeakPIDConstants m_thetaPID;
    protected BeakPIDConstants m_drivePID;

    protected BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();
    protected double m_lastAccel = 0.;

    protected double m_jerk = 0.;

    /**
     * Construct a new generic drivetrain.
     * 
     * @param physics
     *            A {@link RobotPhysics} object containing the
     *            relevant
     *            information for your robot.
     * @param thetaPID
     *            The PID gains for the theta controller.
     * @param drivePID
     *            The PID gains for the auton drive controller.
     */
    public BeakDrivetrain(
        RobotPhysics physics,
        BeakPIDConstants thetaPID,
        BeakPIDConstants drivePID,
        boolean gyroInverted) {
        super(gyroInverted);

        Physics = physics;

        final TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(
            physics.MaxAngularVelocity.in(Units.RadiansPerSecond), physics.MaxAngularVelocity.in(Units.RadiansPerSecond));

        m_thetaController = new ProfiledPIDController(
            thetaPID.kP,
            thetaPID.kI,
            thetaPID.kD,
            thetaConstraints);

        m_thetaPID = thetaPID;
        m_drivePID = drivePID;
    }

    public void configMotors() {
    }

    /**
     * Run a path to the specified point on the field, avoiding obstacles in the process.
     */
    public PathfindingCommand pathFindingCommand(Supplier<Pose2d> desiredPose) {
        return null;
    }

    /**
     * Create a PID controller for preplanned autonomous paths.
     * 
     * @return A {@link PIDController} with the configured values for driving.
     */
    public PIDController createDriveController() {
        return new PIDController(
            m_drivePID.kP,
            m_drivePID.kI,
            m_drivePID.kD);
    }

    /**
     * Create a Profiled PID controller for the rotation of the robot.
     * 
     * @return A {@link PIDController} with the configured values.
     */
    public ProfiledPIDController createThetaController() {
        final TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(
            Physics.MaxAngularVelocity.in(Units.RadiansPerSecond),
            Physics.MaxAngularVelocity.in(Units.RadiansPerSecond));

        return new ProfiledPIDController(
            m_thetaPID.kP,
            m_thetaPID.kI,
            m_thetaPID.kD,
            thetaConstraints);
    }

    public PIDController createAutonThetaController() {
        return new PIDController(
            m_thetaPID.kP,
            m_thetaPID.kI,
            m_thetaPID.kD);
    }

    public SimpleMotorFeedforward getFeedforward() {
        return Physics.Feedforward;
    }

    /**
     * Gets a command to control the
     * drivetrain to follow a path.
     * 
     * @param traj
     *            Trajectory to follow.
     * @return A {@link Command} to run the trajectory, and stop the
     *         drivetrain.
     */
    public Command getTrajectoryCommand(PathPlannerTrajectory traj, Map<String, Command> eventMap) {
        return null;
    }

    /**
     * Create a pathfinding command.
     * @param desiredPose The pose you wish to run to.
     * @param scale How fast the robot should go (1.0 = full speed)
     * @return
     */
    public PathfindingCommand pathFindingCommand(Pose2d desiredPose, double scale) {
        return null;
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param x
     *            Speed of the robot in the x direction (forward).
     * @param y
     *            Speed of the robot in the y direction (sideways).
     * @param rot
     *            Angular rate of the robot.
     * @param fieldRelative
     *            Whether or not the x and y speeds are relative to
     *            the field, for holonomic drivetrains.
     */
    public void drive(double x, double y, double rot, boolean fieldRelative) {
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param x
     *            Speed of the robot in the x direction (forward).
     * @param y
     *            Speed of the robot in the y direction (sideways).
     * @param rot
     *            Angular rate of the robot.
     */
    public void drive(double x, double y, double rot) {
        drive(x, y, rot, false);
    }

    /**
     * Method to drive the robot using chassis speeds
     * 
     * @param speeds
     *            The ChassisSpeeds to use.
     */
    public void drive(ChassisSpeeds speeds) {
    }

    /**
     * Get the current speed of the robot.
     * @return {@link ChassisSpeeds} for the robot.
     */
    public ChassisSpeeds getChassisSpeeds() {
        return null;
    }

    /**
     * Get the robot's pose.
     * 
     * @return The pose reported from the odometry, measured in meters.
     */
    public Pose2d getPoseMeters() {
        return null;
    }

    /**
     * Get the robot's rotation.
     * 
     * @return A {@link Rotation2d} of the reported robot rotation from the
     *         odometry.
     */
    public Rotation2d getRotation2d() {
        return getPoseMeters().getRotation();
    }

    /**
     * Get the robot's heading.
     * 
     * @return The heading reported from the odometry, in degrees.
     */
    public double getHeading() {
        return getRotation2d().getDegrees();
    }

    /**
     * Reset odometry to specified pose.
     * 
     * @param pose
     *            Pose to reset odometry to.
     */
    public void resetOdometry(Pose2d pose) {
    }

    /**
     * Update the robot odometry.
     * 
     * @return Updated pose.
     */
    public Pose2d updateOdometry() {
        return m_pose;
    }

    /**
     * Add a vision measurement to the pose estimator's Kalman filter.
     * 
     * @param estimatedPose
     *            The latest estimated pose from a vision system.
     * @param timestamp
     *            The timestamp of the measured pose.
     */
    public void addVisionMeasurement(Pose2d estimatedPose, double timestamp) {

    }

    /**
     * Add a vision measurement directly from PhotonVision to the pose estimator's Kalman filter.
     * 
     * @param estimatedPose The {@link EstimatedRobotPose} from PhotonVision.
     */
    public void addVisionMeasurement(EstimatedRobotPose estimatedPose) {
        addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
    }

    /**
     * Get the angle to a target position on the field.
     * </p>
     * 
     * Positions are relative to the bottom-left corner of the field (for Rapid
     * React, the blue alliance HP station)
     * 
     * @param x
     *            The X position of the target.
     * @param y
     *            The Y position of the target.
     * @return A {@link Rotation2d} of the drivetrain's angle to the target
     *         position.
     */
    public Rotation2d getAngleToTargetPosition(Measure<Distance> x, Measure<Distance> y) {
        double xDelta = x.in(Units.Meters) - getPoseMeters().getX();
        double yDelta = y.in(Units.Meters) - getPoseMeters().getY();

        double radiansToTarget = Math.atan2(yDelta, xDelta);

        return new Rotation2d(radiansToTarget);
    }

    /**
     * Determine whether or not this drivetrain is holonomic.
     * 
     * @return Whether or not the drivetrain is holonomic (e.g. swerve)
     */
    public boolean isHolonomic() {
        return false;
    }

    /**
     * Get the jerk
     * @return todo
     */
    public double getJerk() {
        return m_jerk;
    }

    @Override
    public void periodic() {
        m_jerk = (m_accelerometer.getY() - m_lastAccel);
    }
}