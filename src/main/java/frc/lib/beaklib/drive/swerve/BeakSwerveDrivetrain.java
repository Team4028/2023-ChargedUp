// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.drive.swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.beaklib.drive.BeakDrivetrain;
import frc.lib.beaklib.drive.RobotPhysics;
import frc.lib.beaklib.gyro.BeakGyro;
import frc.robot.commands.auton.generation.GeneratePath;

/** Generic Swerve Drivetrain subsystem. */
public class BeakSwerveDrivetrain extends BeakDrivetrain {
    /**
     * The modules in this swerve drivetrain.
     * </p>
     * These are in the same order as passed in the constructor; i.e if the front
     * left
     * module is passed in as the first module, <code>m_modules.get(0)</code> would
     * return the front left module.
     */
    protected List<BeakSwerveModule> m_modules = new ArrayList<BeakSwerveModule>();
    int m_numModules;

    protected SwerveDrivePoseEstimator m_odom;
    protected SwerveDriveKinematics m_kinematics;

    protected RobotPhysics m_physics;

    /**
     * An enum representing the direction the robot is currently trying to snap to.
     */
    public enum SnapDirection {
        NONE(0.), //
        UP(0.), //
        DOWN(180.), //
        LEFT(90.), //
        RIGHT(270.);

        /**
         * The angle of this snap direction.
         */
        public Rotation2d Angle;

        /**
         * @param degrees
         *            The angle of the snap direction, in degrees.
         */
        private SnapDirection(double degrees) {
            this.Angle = Rotation2d.fromDegrees(degrees);
        }
    }

    protected SnapDirection m_snapDirection;

    /**
     * Create a new Swerve drivetrain.
     * 
     * @param physics
     *            {@link RobotPhysics} containing the robot's
     *            physical
     *            details.
     * @param gyro
     *            The gyroscope used by this drivetrain.
     * @param thetaPIDGains
     *            The PID gains for the theta controller.
     * @param drivePIDGains
     *            The PID gains for the auton drive controller.
     * @param generatedDrivePIDGains
     *            The PID gains for generated paths using the
     *            {@link GeneratePath} command.
     * @param configs
     *            Configurations for all swerve modules.
     */
    public BeakSwerveDrivetrain(
        RobotPhysics physics,
        BeakGyro gyro,
        boolean gyroInverted,
        double[] thetaPIDGains,
        double[] drivePIDGains,
        double[] generatedDrivePIDGains,
        SwerveModuleConfiguration... configs) {
        super(physics,
            thetaPIDGains,
            drivePIDGains,
            generatedDrivePIDGains,
            gyroInverted);

        m_physics = physics;

        m_numModules = configs.length;
        Translation2d[] moduleLocations = new Translation2d[m_numModules];

        for (int i = 0; i < m_numModules; i++) {
            BeakSwerveModule module = BeakSwerveModule.fromSwerveModuleConfig(configs[i]);
            m_modules.add(module);
            moduleLocations[i] = configs[i].moduleLocation;
        }

        m_gyro = gyro;

        m_kinematics = new SwerveDriveKinematics(moduleLocations);

        m_odom = new SwerveDrivePoseEstimator(m_kinematics, getGyroRotation2d(), getModulePositions(), new Pose2d());

        m_snapDirection = SnapDirection.NONE;

        resetTurningMotors();
    }

    public void logData() {
        Logger logger = Logger.getInstance();
        logger.recordOutput("Pose", getPoseMeters());
        // logger.recordOutput("Module States", getModuleStates());

        double[] modulePositions = new double[m_numModules];
        double[] moduleVelocities = new double[m_numModules];

        SwerveModulePosition[] moduleRecordedPositions = getModulePositions();
        SwerveModuleState[] moduleRecordedStates = getModuleStates();

        for (int i = 0; i < m_numModules; i++) {
            modulePositions[i] = moduleRecordedPositions[i].distanceMeters;
            moduleVelocities[i] = moduleRecordedStates[i].speedMetersPerSecond;
        }
        logger.recordOutput("Swerve/Module Positions", modulePositions);
        logger.recordOutput("Swerve/Module States", getModuleStates());
        logger.recordOutput("Swerve/Module Angles", getModuleAngles());
    }

    public Command getTrajectoryCommand(PathPlannerTrajectory traj, Map<String, Command> eventMap) {
        Command pathFollowingCommand = new PPSwerveControllerCommand(
            traj,
            this::getPoseMeters,
            createDriveController(),
            createDriveController(),
            createAutonThetaController(),
            this::drive,
            true,
            this);

        return new FollowPathWithEvents(
            pathFollowingCommand,
            traj.getMarkers(),
            eventMap);
    }

    public Command generatePath(Supplier<Pose2d> desiredPose) {
        return new GeneratePath(desiredPose, this).andThen(new InstantCommand(() -> this.drive(0, 0, 0, false)));
    }

    public Pose2d updateOdometry() {
        m_pose = m_odom.updateWithTime(
            Timer.getFPGATimestamp(),
            getGyroRotation2d(),
            getModulePositions());

        return m_pose;
    }

    public void addVisionMeasurement(Pose2d estimatedPose, double timestamp) {
        Transform2d poseError = estimatedPose.minus(m_odom.getEstimatedPosition());

        if (!estimatedPose.equals(new Pose2d()) && !estimatedPose.equals(getPoseMeters()) &&
            Math.abs(poseError.getX()) < 0.5 &&
            Math.abs(poseError.getY()) < 0.5) {
            m_odom.addVisionMeasurement(estimatedPose, timestamp);
        }
    }

    public Pose2d getPoseMeters() {
        return m_odom.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        if (!pose.equals(new Pose2d()))
            m_odom.resetPosition(getGyroRotation2d(), getModulePositions(), pose);
    }

    public void drive(double x, double y, double rot, boolean fieldRelative) {
        x *= m_physics.maxVelocity.getAsMetersPerSecond();
        y *= m_physics.maxVelocity.getAsMetersPerSecond();
        rot *= m_physics.maxAngularVelocity.getAsRadiansPerSecond();

        ChassisSpeeds speeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, getRotation2d())
            : new ChassisSpeeds(x, y, rot);

        drive(speeds);
    }

    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);

        Logger.getInstance().recordOutput("Swerve/Module Setpoints", states);

        setModuleStates(states);
    }

    /* Swerve-specific Methods */

    /**
     * Set each module's {@link SwerveModuleState}.
     * 
     * @param desiredStates
     *            An array of the desired states for the m_modules.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, m_physics.maxVelocity.getAsMetersPerSecond());

        for (int i = 0; i < desiredStates.length; i++) {
            m_modules.get(i).setDesiredState(desiredStates[i]);
        }
    }

    /**
     * Get the states of each module.
     * 
     * @return Array of {@link SwerveModuleState}s for each module.
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[m_numModules];
        for (int i = 0; i < m_numModules; i++) {
            states[i] = m_modules.get(i).getState();
        }

        return states;
    }

    /**
     * Get the positions of each module.
     * 
     * @return Array of {@link SwerveModulePosition}s for each module.
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[m_numModules];
        for (int i = 0; i < m_numModules; i++) {
            states[i] = m_modules.get(i).getPosition();
        }

        return states;
    }

    /**
     * Get the angles of each module.
     * 
     * @return Array of the angles for each module.
     */
    public double[] getModuleAngles() {
        double[] states = new double[m_numModules];
        for (int i = 0; i < m_numModules; i++) {
            states[i] = Units.radiansToDegrees(m_modules.get(i).getTurningEncoderRadians());
        }

        return states;
    }

    /**
     * Reset all drive and turning encoders to zero.
     */
    public void resetEncoders() {
        for (BeakSwerveModule module : m_modules) {
            module.resetEncoders();
        }
    }

    /**
     * Re-zero all turning encoders to match the CANCoder.
     */
    public void resetTurningMotors() {
        for (BeakSwerveModule module : m_modules) {
            module.resetTurningMotor();
        }
    }

    /**
     * Zero the pose and heading of the robot.
     */
    public void zero() {
        resetTurningMotors();
        m_odom.resetPosition(getGyroRotation2d(), getModulePositions(), new Pose2d());
    }

    private ChassisSpeeds getChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(
            getModuleStates());
    }

    /**
     * Get the forward velocity of the drivetrain.
     * 
     * @return The X (forward) velocity of the drivetrain, in meters per second.
     */
    public double getForwardVelocity() {
        return getChassisSpeeds().vxMetersPerSecond;
    }

    /**
     * Get the sideways velocity of the drivetrain.
     * 
     * @return The Y (sideways) velocity of the drivetrain, in meters per second.
     */
    public double getSidewaysVelocity() {
        return getChassisSpeeds().vyMetersPerSecond;
    }

    /**
     * Get the angular velocity of the drivetrain.
     * 
     * @return The angular velocity of the drivetrain, in radians per second.
     */
    public double getAngularVelocity() {
        return getChassisSpeeds().omegaRadiansPerSecond;
    }

    /**
     * Set the snap direction of the robot.
     * 
     * @param newDirection
     *            The direction the robot should snap to.
     */
    public void setSnapDirection(SnapDirection newDirection) {
        m_snapDirection = newDirection;
    }

    public boolean isHolonomic() {
        return true;
    }

    @Override
    public void periodic() {
        super.periodic();
        
        updateOdometry();
        logData();
        SmartDashboard.putString("Snap Direction", m_snapDirection.name());
    }
}
