// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import frc.lib.beaklib.drive.RobotPhysics;
import frc.lib.beaklib.drive.swerve.BeakSwerveDrivetrain;
import frc.lib.beaklib.drive.swerve.SdsModuleConfiguration;
import frc.lib.beaklib.drive.swerve.SdsModuleConfigurations;
import frc.lib.beaklib.drive.swerve.SwerveDrivetrainConfiguration;
import frc.lib.beaklib.drive.swerve.SwerveModuleConfiguration;
import frc.lib.beaklib.gyro.BeakPigeon2;
import frc.lib.beaklib.units.Acceleration;
import frc.lib.beaklib.units.AngularVelocity;
import frc.lib.beaklib.units.Distance;
import frc.lib.beaklib.units.Velocity;
import frc.robot.Constants.PIDConstants;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class PracticeSwerveDrivetrain extends BeakSwerveDrivetrain {
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

    private static final double DRIVE_kP = 0.0125;
    private static final double TURN_kP = 0.2;
    private static final double TURN_kD = 0.0;

    private static final double AUTON_kP = 3.;
    private static final double[] AUTON_DRIVE_GAINS = { AUTON_kP, 0., 0. };

    private static final double GENERATED_AUTON_kP = 7.5;
    private static final double[] GENERATED_AUTON_DRIVE_GAINS = { GENERATED_AUTON_kP, 0., 0.01 };

    private static final int PIGEON2_ID = 1;
    private static final String CAN_BUS = "rio";

    private static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(
        0.,
        0.,
        0.);

    private static final SdsModuleConfiguration CONFIGURATION = SdsModuleConfigurations.UNCHARACTERIZED_MK4I_L2;

    private static final Velocity MAX_VELOCITY = Velocity.fromFeetPerSecond(16.3);
    private static final Acceleration MAX_ACCEL = Acceleration.fromFeetPerSecondSquared(16.3);

    // distance from the right to left wheels on the robot
    private static final Distance TRACK_WIDTH = Distance.fromInches(26.);
    // distance from the front to back wheels on the robot
    private static final Distance WHEEL_BASE = Distance.fromInches(28.);

    private static final RobotPhysics PHYSICS = new RobotPhysics(
        MAX_VELOCITY,
        new AngularVelocity(),
        MAX_ACCEL,
        TRACK_WIDTH,
        WHEEL_BASE,
        CONFIGURATION.wheelDiameter,
        CONFIGURATION.driveGearRatio,
        FEED_FORWARD);

    private static PracticeSwerveDrivetrain m_instance;

    private Field2d m_field = new Field2d();

    private static final int FL_DRIVE_ID = 2;
    private static final int FL_TURN_ID = 1;
    private static final int FL_ENCODER_ID = 1; // SHOULD BE 9
    private static final double FL_OFFSET = -Units.degreesToRadians(180.9);
    private static final Translation2d FL_LOCATION = new Translation2d(WHEEL_BASE.getAsMeters() / 2,
        TRACK_WIDTH.getAsMeters() / 2); // TODO: Please God BeakTranslation2d

    private static final int FR_DRIVE_ID = 4;
    private static final int FR_TURN_ID = 3;
    private static final int FR_ENCODER_ID = 2; // SHOULD BE 10
    private static final double FR_OFFSET = -Math.toRadians(86.8);
    private static final Translation2d FR_LOCATION = new Translation2d(WHEEL_BASE.getAsMeters() / 2,
        -TRACK_WIDTH.getAsMeters() / 2);

    private static final int BL_DRIVE_ID = 6;
    private static final int BL_TURN_ID = 5;
    private static final int BL_ENCODER_ID = 3; // SHOULD BE 11
    private static final double BL_OFFSET = -Math.toRadians(328.7);
    private static final Translation2d BL_LOCATION = new Translation2d(-WHEEL_BASE.getAsMeters() / 2,
        TRACK_WIDTH.getAsMeters() / 2);

    private static final int BR_DRIVE_ID = 8;
    private static final int BR_TURN_ID = 7;
    private static final int BR_ENCODER_ID = 4; // SHOULD BE 12
    private static final double BR_OFFSET = -Math.toRadians(159.6);
    private static final Translation2d BR_LOCATION = new Translation2d(-WHEEL_BASE.getAsMeters() / 2,
        -TRACK_WIDTH.getAsMeters() / 2);

    private static final double ALLOWED_CLOSED_LOOP_ERROR = 40.0;

    private static final int TURN_CURRENT_LIMIT = 20;
    private static final int DRIVE_SUPPLY_LIMIT = 60;
    private static final int DRIVE_STATOR_LIMIT = 80;

    private final static BeakPigeon2 m_gyro = new BeakPigeon2(PIGEON2_ID, CAN_BUS);

    private static final SwerveDrivetrainConfiguration DRIVE_CONFIG = new SwerveDrivetrainConfiguration(
        DRIVE_kP,
        TURN_kP,
        TURN_kD,
        ALLOWED_CLOSED_LOOP_ERROR,
        TURN_CURRENT_LIMIT,
        DRIVE_SUPPLY_LIMIT,
        DRIVE_STATOR_LIMIT,
        CAN_BUS,
        FEED_FORWARD,
        CONFIGURATION);

    private static SwerveModuleConfiguration m_frontLeftConfig = new SwerveModuleConfiguration(
        FL_DRIVE_ID,
        FL_TURN_ID,
        FL_ENCODER_ID,
        FL_OFFSET,
        FL_LOCATION,
        DRIVE_CONFIG);

    private static SwerveModuleConfiguration m_frontRightConfig = new SwerveModuleConfiguration(
        FR_DRIVE_ID,
        FR_TURN_ID,
        FR_ENCODER_ID,
        FR_OFFSET,
        FR_LOCATION,
        DRIVE_CONFIG);

    private static SwerveModuleConfiguration m_backLeftConfig = new SwerveModuleConfiguration(
        BL_DRIVE_ID,
        BL_TURN_ID,
        BL_ENCODER_ID,
        BL_OFFSET,
        BL_LOCATION,
        DRIVE_CONFIG);

    private static SwerveModuleConfiguration m_backRightConfig = new SwerveModuleConfiguration(
        BR_DRIVE_ID,
        BR_TURN_ID,
        BR_ENCODER_ID,
        BR_OFFSET,
        BR_LOCATION,
        DRIVE_CONFIG);

    public PracticeSwerveDrivetrain() {
        super(
            PHYSICS,
            m_gyro,
            false,
            PIDConstants.Theta.gains,
            AUTON_DRIVE_GAINS,
            GENERATED_AUTON_DRIVE_GAINS,
            m_frontLeftConfig,
            m_frontRightConfig,
            m_backLeftConfig,
            m_backRightConfig);

        m_odom = new SwerveDrivePoseEstimator(
            m_kinematics,
            getGyroRotation2d(),
            getModulePositions(),
            new Pose2d(),
            m_stateStdDevs,
            m_visionMeasurementStdDevs);
    }

    public static PracticeSwerveDrivetrain getInstance() {
        if (m_instance == null) {
            m_instance = new PracticeSwerveDrivetrain();
        }
        return m_instance;
    }

    @Override
    public void periodic() {
        updateOdometry();

        m_field.setRobotPose(getPoseMeters());
        SmartDashboard.putData(m_field);

        logData();

        SmartDashboard.putNumber("Pitch", getGyroRollRotation2d().getDegrees());
    }
}