// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import frc.lib.beaklib.drive.RobotPhysics;
import frc.lib.beaklib.drive.swerve.BeakSwerveDrivetrain;
import frc.lib.beaklib.drive.swerve.SdsModuleConfiguration;
import frc.lib.beaklib.drive.swerve.SdsModuleConfigurations;
import frc.lib.beaklib.drive.swerve.DrivetrainConfiguration;
import frc.lib.beaklib.drive.swerve.SwerveModuleConfiguration;
import frc.lib.beaklib.gyro.BeakPigeon2;
import frc.lib.beaklib.pid.BeakPIDConstants;
import frc.lib.beaklib.units.Acceleration;
import frc.lib.beaklib.units.AngularVelocity;
import frc.lib.beaklib.units.Distance;
import frc.lib.beaklib.units.Velocity;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class SwerveDrivetrain extends BeakSwerveDrivetrain {
    // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
    // you trust your various sensors. Smaller numbers will cause the filter to
    // "trust" the estimate from that particular component more than the others.
    // This in turn means the particualr component will have a stronger influence
    // on the final pose estimate.

    /**
     * Standard deviations of model states. Increase these numbers to trust your
     * model's state estimates less. This
     * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
     * meters.
     */
    private static final Vector<N3> m_stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

    /**
     * Standard deviations of the vision measurements. Increase these numbers to
     * trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
     * radians.
     */
    private static final Vector<N3> m_visionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1,
        Units.degreesToRadians(25));

    private static final BeakPIDConstants DRIVE_PID = new BeakPIDConstants(0.03);
    private static final BeakPIDConstants TURN_PID = new BeakPIDConstants(0.15);

    private static final BeakPIDConstants AUTON_DRIVE_PID = new BeakPIDConstants(4.0, 0., 0.01);
    private static final BeakPIDConstants GENERATED_AUTON_DRIVE_PID = new BeakPIDConstants(7.5, 0., 0.01);
    private static final BeakPIDConstants AUTON_THETA_PID = new BeakPIDConstants(5.0, 0., 0.2);

    private static final int PIGEON2_ID = 0;
    private static final String CAN_BUS = "rio";

    private static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(
        (.2212 + .151 + .163) / 3.,
        (2.3 + 2.32 + 2.33) / 3.,
        (.421 + .849 + .708) / 3.);

    private static final SdsModuleConfiguration CONFIGURATION = SdsModuleConfigurations.MK4I_L2;

    private static final Velocity MAX_VELOCITY = Velocity.fromFeetPerSecond(16.3);
    private static final Acceleration MAX_ACCEL = Acceleration.fromFeetPerSecondSquared(16.3);

    // distance from the right to left wheels on the robot
    private static final Distance TRACK_WIDTH = Distance.fromInches(19.688); // 24.
    // distance from the front to back wheels on the robot
    private static final Distance WHEEL_BASE = Distance.fromInches(23.688); // 28.

    private static final RobotPhysics PHYSICS = new RobotPhysics(
        MAX_VELOCITY,
        new AngularVelocity(),
        MAX_ACCEL,
        TRACK_WIDTH,
        WHEEL_BASE,
        CONFIGURATION.wheelDiameter,
        CONFIGURATION.driveGearRatio,
        FEED_FORWARD);

    private static SwerveDrivetrain m_instance;

    private Field2d m_field = new Field2d();

    private static final int FL_DRIVE_ID = 2;
    private static final int FL_TURN_ID = 1;
    private static final int FL_ENCODER_ID = 1; // SHOULD BE 9
    private static final Rotation2d FL_OFFSET = Rotation2d.fromDegrees(-357.4);
    private static final Translation2d FL_LOCATION = new Translation2d(WHEEL_BASE.getAsMeters() / 2,
        TRACK_WIDTH.getAsMeters() / 2); // TODO: Please God BeakTranslation2d

    private static final int FR_DRIVE_ID = 4;
    private static final int FR_TURN_ID = 3;
    private static final int FR_ENCODER_ID = 2; // SHOULD BE 10
    private static final Rotation2d FR_OFFSET = Rotation2d.fromDegrees(-271.3);
    private static final Translation2d FR_LOCATION = new Translation2d(WHEEL_BASE.getAsMeters() / 2,
        -TRACK_WIDTH.getAsMeters() / 2);

    private static final int BL_DRIVE_ID = 6;
    private static final int BL_TURN_ID = 5;
    private static final int BL_ENCODER_ID = 3; // SHOULD BE 11
    private static final Rotation2d BL_OFFSET = Rotation2d.fromDegrees(-327.3);
    private static final Translation2d BL_LOCATION = new Translation2d(-WHEEL_BASE.getAsMeters() / 2,
        TRACK_WIDTH.getAsMeters() / 2);

    private static final int BR_DRIVE_ID = 8;
    private static final int BR_TURN_ID = 7;
    private static final int BR_ENCODER_ID = 4; // SHOULD BE 12
    private static final Rotation2d BR_OFFSET = Rotation2d.fromDegrees(-160.5);
    private static final Translation2d BR_LOCATION = new Translation2d(-WHEEL_BASE.getAsMeters() / 2,
        -TRACK_WIDTH.getAsMeters() / 2);

    private static final double ALLOWED_CLOSED_LOOP_ERROR = 40.0;

    private static final int TURN_CURRENT_LIMIT = 20;
    private static final int DRIVE_SUPPLY_LIMIT = 75;
    private static final int DRIVE_STATOR_LIMIT = 80;

    private final static BeakPigeon2 m_gyro = new BeakPigeon2(PIGEON2_ID, CAN_BUS);

    private static final DrivetrainConfiguration DRIVE_CONFIG = new DrivetrainConfiguration(
        DRIVE_PID,
        TURN_PID,
        false,
        ALLOWED_CLOSED_LOOP_ERROR,
        TURN_CURRENT_LIMIT,
        DRIVE_SUPPLY_LIMIT,
        DRIVE_STATOR_LIMIT,
        CAN_BUS,
        FEED_FORWARD,
        PHYSICS);

    private static SwerveModuleConfiguration m_frontLeftConfig = new SwerveModuleConfiguration(
        FL_OFFSET,
        FL_LOCATION,
        CONFIGURATION.driveGearRatio,
        CONFIGURATION.turnGearRatio,
        CONFIGURATION.wheelDiameter,
        CONFIGURATION.driveInverted,
        CONFIGURATION.turnInverted,
        DRIVE_CONFIG);

    private static SwerveModuleConfiguration m_frontRightConfig = new SwerveModuleConfiguration(
        FR_OFFSET,
        FR_LOCATION,
        CONFIGURATION.driveGearRatio,
        CONFIGURATION.turnGearRatio,
        CONFIGURATION.wheelDiameter,
        CONFIGURATION.driveInverted,
        CONFIGURATION.turnInverted,
        DRIVE_CONFIG);

    private static SwerveModuleConfiguration m_backLeftConfig = new SwerveModuleConfiguration(
        BL_OFFSET,
        BL_LOCATION,
        CONFIGURATION.driveGearRatio,
        CONFIGURATION.turnGearRatio,
        CONFIGURATION.wheelDiameter,
        CONFIGURATION.driveInverted,
        CONFIGURATION.turnInverted,
        DRIVE_CONFIG);

    private static SwerveModuleConfiguration m_backRightConfig = new SwerveModuleConfiguration(
        BR_OFFSET,
        BR_LOCATION,
        CONFIGURATION.driveGearRatio,
        CONFIGURATION.turnGearRatio,
        CONFIGURATION.wheelDiameter,
        CONFIGURATION.driveInverted,
        CONFIGURATION.turnInverted,
        DRIVE_CONFIG);

    public SwerveDrivetrain() {
        super(
            PHYSICS,
            m_gyro,
            false,
            AUTON_THETA_PID,
            AUTON_DRIVE_PID,
            GENERATED_AUTON_DRIVE_PID);

        super.setup(
            new MK4iSwerveModule(
                FL_DRIVE_ID,
                FL_TURN_ID,
                FL_ENCODER_ID,
                m_frontLeftConfig),
            new MK4iSwerveModule(
                FR_DRIVE_ID,
                FR_TURN_ID,
                FR_ENCODER_ID,
                m_frontRightConfig),
            new MK4iSwerveModule(
                BL_DRIVE_ID,
                BL_TURN_ID,
                BL_ENCODER_ID,
                m_backLeftConfig),
            new MK4iSwerveModule(
                BR_DRIVE_ID,
                BR_TURN_ID,
                BR_ENCODER_ID,
                m_backRightConfig) //
            );

        m_odom = new SwerveDrivePoseEstimator(
            m_kinematics,
            getGyroRotation2d(),
            getModulePositions(),
            new Pose2d(),
            m_stateStdDevs,
            m_visionMeasurementStdDevs);
    }

    public static SwerveDrivetrain getInstance() {
        if (m_instance == null) {
            m_instance = new SwerveDrivetrain();
        }
        return m_instance;
    }

    @Override
    public void periodic() {
        super.periodic();

        m_field.setRobotPose(getPoseMeters());
        SmartDashboard.putData(m_field);

        SmartDashboard.putNumber("Pitch", getGyroPitchRotation2d().getDegrees());

        SmartDashboard.putNumber("FL angle", Math.toDegrees(m_modules.get(0).getAbsoluteEncoderRadians()));
        SmartDashboard.putNumber("FR angle", Math.toDegrees(m_modules.get(1).getAbsoluteEncoderRadians()));
        SmartDashboard.putNumber("BL angle", Math.toDegrees(m_modules.get(2).getAbsoluteEncoderRadians()));
        SmartDashboard.putNumber("BR angle", Math.toDegrees(m_modules.get(3).getAbsoluteEncoderRadians()));

        SmartDashboard.putNumber("X (meters)", m_odom.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Y (meters)", m_odom.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Heading (deg)", getHeading());

        SmartDashboard.putNumber("Velocity", super.getForwardVelocity());
    }
}