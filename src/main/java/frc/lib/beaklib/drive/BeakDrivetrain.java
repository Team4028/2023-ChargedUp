// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.beaklib.pid.BeakPIDConstants;
import frc.lib.beaklib.subsystem.BeakGyroSubsystem;
import frc.lib.beaklib.units.Acceleration;
import frc.lib.beaklib.units.AngularVelocity;
import frc.lib.beaklib.units.Distance;
import frc.lib.beaklib.units.Velocity;

/** Base drivetrain class. */
public class BeakDrivetrain extends BeakGyroSubsystem {
    protected Pose2d m_pose;

    protected Velocity m_maxVelocity;
    protected AngularVelocity m_maxAngularVelocity;
    protected Acceleration m_maxAccel;

    protected Distance m_trackWidth;
    protected Distance m_wheelBase;

    protected Distance m_wheelDiameter;

    protected double m_gearRatio;

    protected SimpleMotorFeedforward m_feedForward;

    protected ProfiledPIDController m_thetaController;
    protected BeakPIDConstants m_thetaPID;
    protected BeakPIDConstants m_drivePID;
    protected BeakPIDConstants m_generatedDrivePID;

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
     * @param generatedDrivePID
     *            The PID gains for generated paths using a path generation command.
     */
    public BeakDrivetrain(
        RobotPhysics physics,
        BeakPIDConstants thetaPID,
        BeakPIDConstants drivePID,
        BeakPIDConstants generatedDrivePID,
        boolean gyroInverted) {
        super(gyroInverted);
        m_maxVelocity = physics.maxVelocity;
        m_maxAngularVelocity = physics.maxAngularVelocity;
        m_maxAccel = physics.maxAccel;

        m_trackWidth = physics.trackWidth;
        m_wheelBase = physics.wheelBase;
        m_wheelDiameter = physics.wheelDiameter;

        m_gearRatio = physics.driveGearRatio;
        m_feedForward = physics.feedforward;

        final TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(
            physics.maxAngularVelocity.getAsRadiansPerSecond(), physics.maxAngularVelocity.getAsRadiansPerSecond());

        m_thetaController = new ProfiledPIDController(
            thetaPID.kP,
            thetaPID.kI,
            thetaPID.kD,
            thetaConstraints);

        m_thetaPID = thetaPID;
        m_drivePID = drivePID;
        m_generatedDrivePID = generatedDrivePID;
    }

    public void configMotors() {
    }

    public RobotPhysics getPhysics() {
        return new RobotPhysics(
            m_maxVelocity,
            m_maxAngularVelocity,
            m_maxAccel,
            m_trackWidth,
            m_wheelBase,
            m_wheelDiameter,
            m_gearRatio,
            m_feedForward);
    }

    /**
     * Generate a runnable trajectory from the current robot pose to the desired
     * pose.
     * 
     * @return {@link PathPlannerTrajectory} that leads the robot to the desired
     *         pose.
     */
    public PathPlannerTrajectory generateTrajectoryToPose(Pose2d desiredPose, double velocityMultiplier) {
        Pose2d robotPose = getPoseMeters();

        // If the current position is equal to the target position (for example, no
        // target is visible if using vision pose), return a blank trajectory that does
        // nothing. This saves computational time and is generally good practice.
        if (robotPose.equals(desiredPose)) {
            return new PathPlannerTrajectory();
        }

        // PathPlanner takes in these constraints to determine maximum speed and
        // acceleration.
        PathConstraints constraints = new PathConstraints(getPhysics().maxVelocity.getAsMetersPerSecond() * velocityMultiplier,
            getPhysics().maxVelocity.getAsMetersPerSecond() * velocityMultiplier);

        // Add our path points--start at the current robot pose and end at the desired
        // pose.
        List<PathPoint> points = new ArrayList<PathPoint>();

        points.add(new PathPoint(robotPose.getTranslation(), robotPose.getRotation(), robotPose.getRotation()));
        points.add(new PathPoint(desiredPose.getTranslation(), desiredPose.getRotation(), desiredPose.getRotation()));

        // PathPlanner has a built in path generation function!
        PathPlannerTrajectory traj = PathPlanner.generatePath(constraints, points);

        return traj;
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
     * Create a PID controller for on-the-fly generated autonomous paths.
     * 
     * @return A {@link PIDController} with the configured values for generated
     *         driving.
     */
    public PIDController createGeneratedDriveController() {
        return new PIDController(
            m_generatedDrivePID.kP,
            m_generatedDrivePID.kI,
            m_generatedDrivePID.kD);
    }

    /**
     * Create a Profiled PID controller for the rotation of the robot.
     * 
     * @return A {@link PIDController} with the configured values.
     */
    public ProfiledPIDController createThetaController() {
        final TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(
            getPhysics().maxAngularVelocity.getAsRadiansPerSecond(),
            getPhysics().maxAngularVelocity.getAsRadiansPerSecond());

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
        return m_feedForward;
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
     * Gets a command to generate a path to the desired pose and follow that path.
     * 
     * @param desiredPose
     *            A supplier of the pose to run to. When the command is
     *            initialized, this should evaluate to the final desired
     *            pose.
     * @return A {@link Command} that runs the generated path.
     */
    public Command generatePath(Supplier<Pose2d> desiredPose) {
        return Commands.none();
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
     *            The X position of the target, in inches.
     * @param y
     *            The Y position of the target, in inches.
     * @return A {@link Rotation2d} of the drivetrain's angle to the target
     *         position.
     */
    public Rotation2d getAngleToTargetPosition(Distance x, Distance y) {
        double xDelta = x.getAsMeters() - getPoseMeters().getX();
        double yDelta = y.getAsMeters() - getPoseMeters().getY();

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