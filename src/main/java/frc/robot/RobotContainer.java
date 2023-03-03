// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.beaklib.BeakXBoxController;
import frc.lib.beaklib.Util;
import frc.lib.beaklib.drive.swerve.BeakSwerveDrivetrain;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.arms.Arm;
import frc.robot.subsystems.arms.LowerArm;
import frc.robot.subsystems.arms.UpperArm;
import frc.robot.subsystems.infeed.Infeed;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.Wrist;
import frc.robot.commands.arm.CurrentZero;
import frc.robot.commands.arm.RunArmsToPosition;
import frc.robot.commands.auton.Autons;
import frc.robot.commands.auton.BeakAutonCommand;
import frc.robot.commands.chassis.AddVisionMeasurement;
import frc.robot.commands.chassis.AutoBalance;
import frc.robot.commands.chassis.ResetPoseToVision;
import frc.robot.subsystems.swerve.PoseEstimatorSwerveDrivetrain;
import frc.robot.subsystems.swerve.PracticeSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.utilities.Trajectories.PathPosition;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Constants
    private static final String FRONT_APRILTAG_CAMERA_NAME = Constants.PRACTICE_CHASSIS ? "Front_AprilTag_Camera"
        : "Global_Shutter_Camera";
    private static final String REAR_APRILTAG_CAMERA_NAME = "Rear_AprilTag_Camera";
    // private static final String GAME_PIECE_CAMERA_NAME = "HD_Webcam_C525"; //
    // Very much subject to change.

    private static final Pose3d FRONT_APRILTAG_CAMERA_TO_ROBOT = Constants.PRACTICE_CHASSIS
        ? new Pose3d(Units.inchesToMeters(5.),
            Units.inchesToMeters(6.), 0.,
            new Rotation3d(0., Units.degreesToRadians(11.0),
                Units.degreesToRadians(180.)))
        : new Pose3d(Units.inchesToMeters(-2.),
            Units.inchesToMeters(-2.), 0.,
            new Rotation3d(0., Units.degreesToRadians(11.0), Units.degreesToRadians(-8.)));

    private static final Pose3d REAR_APRILTAG_CAMERA_TO_ROBOT = new Pose3d(Units.inchesToMeters(6.),
        Units.inchesToMeters(6.), 0.,
        new Rotation3d(0., Units.degreesToRadians(16.0), Units.degreesToRadians(0.)));
    // private static final Pose3d GAME_PIECE_CAMERA_TO_ROBOT = new
    // Pose3d(Units.inchesToMeters(12.), 0., 0., new Rotation3d());

    // Subsystems
    private final BeakSwerveDrivetrain m_drive;
    private final Vision m_frontAprilTagVision;
    private final Vision m_rearAprilTagVision;
    private final UpperArm m_upperArm;
    private final LowerArm m_lowerArm;

    private final Manipulator m_manipulator;
    private final Infeed m_infeed;
    private final Wrist m_wrist;

    // Controller
    private final BeakXBoxController m_driverController = new BeakXBoxController(0);
    // private final BeakXBoxController m_operatorController = new
    // BeakXBoxController(1);

    // Auton stuff
    private final LoggedDashboardChooser<BeakAutonCommand> autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    private final Autons m_autons;
    // private final LoggedDashboardNumber flywheelSpeedInput = new
    // LoggedDashboardNumber("Flywheel Speed", 1500.0);

    // Limiters, etc.
    private SlewRateLimiter m_xLimiter = new SlewRateLimiter(4.0);
    private SlewRateLimiter m_yLimiter = new SlewRateLimiter(4.0);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(4.0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_drive = PracticeSwerveDrivetrain.getInstance();
        // m_drive = PoseEstimatorSwerveDrivetrain.getInstance();
        m_frontAprilTagVision = new Vision(FRONT_APRILTAG_CAMERA_NAME, FRONT_APRILTAG_CAMERA_TO_ROBOT, false);
        m_rearAprilTagVision = new Vision(REAR_APRILTAG_CAMERA_NAME, REAR_APRILTAG_CAMERA_TO_ROBOT, false);

        if (Constants.PRACTICE_CHASSIS) {
            m_manipulator = Manipulator.getInstance();
            m_infeed = Infeed.getInstance();
            m_wrist = Wrist.getInstance();

            m_upperArm = UpperArm.getInstance();
            m_lowerArm = LowerArm.getInstance();
        } else {
            m_manipulator = null;
            m_infeed = null;
            m_wrist = null;

            m_upperArm = null;
            m_lowerArm = null;
        }

        RobotState.addSubsystems(null, m_drive, m_frontAprilTagVision);

        m_autons = new Autons(m_drive, m_lowerArm, m_frontAprilTagVision, m_rearAprilTagVision);

        switch (Constants.currentMode) {
            // TODO
            // Real robot, instantiate hardware IO implementations
            case REAL:
                // drive = new Drive(new DriveIOSparkMax());
                // flywheel = new Flywheel(new FlywheelIOSparkMax());
                // drive = new Drive(new DriveIOFalcon500());
                // flywheel = new Flywheel(new FlywheelIOFalcon500());
                break;

            // Sim robot, instantiate physics sim IO implementations
            case SIM:
                // drive = new Drive(new DriveIOSim());
                // flywheel = new Flywheel(new FlywheelIOSim());
                break;

            // Replayed robot, disable IO implementations
            default:
                // drive = new Drive(new DriveIO() {
                // });
                // flywheel = new Flywheel(new FlywheelIO() {
                // });
                break;
        }

        // Set up auto routines
        // autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
        // autoChooser.addOption("Spin", new SpinAuto(drive));
        // autoChooser.addOption("Drive With Flywheel", new DriveWithFlywheelAuto(drive,
        // flywheel));
        // autoChooser.addOption("Drive With Flywheel", new DriveWithFlywheelAuto(drive,
        // flywheel));

        // Configure the button bindings
        configureButtonBindings();
        initAutonChooser();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        m_drive.setDefaultCommand(
            new RunCommand(() -> m_drive.drive(
                -speedScaledDriverLeftY(),
                speedScaledDriverLeftX(),
                speedScaledDriverRightX(),
                true),
                m_drive));

        if (Constants.PRACTICE_CHASSIS) {
            m_driverController.b.onTrue(new ConditionalCommand(
                new RunArmsToPosition(Arm.ArmPositions.ACQUIRE_FLOOR, Wrist.WristPositions.INFEED_CONE,
                    m_lowerArm, m_upperArm, m_wrist),
                new RunArmsToPosition(Arm.ArmPositions.ACQUIRE_FLOOR, Wrist.WristPositions.INFEED_CUBE,
                    m_lowerArm, m_upperArm, m_wrist),
                () -> RobotState.getState() == RobotState.State.CONE));

            m_driverController.x
                .onTrue(new RunArmsToPosition(Arm.ArmPositions.SCORE_MID, Wrist.WristPositions.SCORE_MID,
                    m_lowerArm, m_upperArm, m_wrist));
        } else {
            m_driverController.b.onTrue(new InstantCommand(() -> RobotState.modeCone()));
            m_driverController.x.onTrue(new InstantCommand(() -> RobotState.modeCube()));
        }
        m_driverController.y.onTrue(new ResetPoseToVision(m_drive, m_frontAprilTagVision));

        m_driverController.rb.onTrue(new AutoBalance(m_drive));

        m_driverController.lb.onTrue(new InstantCommand(() -> RobotState.toggleAutoAlign())
            .andThen(new ResetPoseToVision(m_drive, m_frontAprilTagVision)
                .andThen(RobotState.setNodeFromTagID(() -> m_frontAprilTagVision.getLatestTagID()))));

        m_driverController.dpadRight.onTrue(RobotState.incrementNode());
        m_driverController.dpadLeft.onTrue(RobotState.decrementNode());
        m_driverController.dpadDown.onTrue(RobotState.runToNodePosition());
        m_driverController.dpadUp.onTrue(RobotState.setNodeFromTagID(() -> m_frontAprilTagVision.getLatestTagID()));

        if (Constants.PRACTICE_CHASSIS) {
            m_driverController.back
                .onTrue(new CurrentZero(m_upperArm, -0.2).andThen(new CurrentZero(m_lowerArm, -0.1)));
        }
        m_driverController.start.onTrue(new InstantCommand(m_drive::zero));

        // // infeed
        // m_operatorController.rb.onTrue(m_infeed.runMotorIn());
        // m_operatorController.rb.onFalse(m_infeed.stopMotor());
        // m_operatorController.lb.onTrue(m_infeed.runMotorOut());
        // m_operatorController.lb.onFalse(m_infeed.stopMotor());

        // // wrist
        // m_operatorController.rt.onTrue(m_wrist.runMotorUp());
        // m_operatorController.rt.onFalse(m_wrist.stopMotor());
        // m_operatorController.lt.onTrue(m_wrist.runMotorDown());
        // m_operatorController.lt.onFalse(m_wrist.stopMotor());

        // // mode
        // m_operatorController.a.onTrue(new InstantCommand(() ->
        // RobotState.toggleClimb()));
        // m_operatorController.x.onTrue(new InstantCommand(() ->
        // RobotState.modeCube()));
        // m_operatorController.y.onTrue(new InstantCommand(() ->
        // RobotState.modeCone()));
        // m_operatorController.b.onTrue(new InstantCommand(() ->
        // RobotState.modeBlank()));
    }

    private void initAutonChooser() {
        autoChooser.addDefaultOption("j path 1", m_autons.JPath1());

        // autoChooser.addOption("j path 2", new JPath2(m_drive));
        // autoChooser.addOption("J Path", new JPath(m_drive));

        autoChooser.addOption("Two Piece Top", m_autons.TwoPiece(PathPosition.TOP));
        autoChooser.addOption("Two Piece Top Acquire", m_autons.TwoPieceAcquire(PathPosition.TOP));
        autoChooser.addOption("Two Piece Top Score", m_autons.TwoPieceScore(PathPosition.TOP));
        autoChooser.addOption("Two Piece Bottom", m_autons.TwoPiece(PathPosition.BOTTOM));
        autoChooser.addOption("Two Piece Bottom Acquire", m_autons.TwoPieceAcquire(PathPosition.BOTTOM));
        autoChooser.addOption("Two Piece Bottom Score", m_autons.TwoPieceScore(PathPosition.BOTTOM));
    }

    public double speedScaledDriverLeftY() {
        return m_yLimiter.calculate(Util.speedScale(m_driverController.getLeftYAxis(),
            RobotState.getAutoAlign() ? DriveConstants.AUTO_ALIGN_SPEED_SCALE : DriveConstants.SPEED_SCALE,
            m_driverController.getRightTrigger()));
    }

    public double speedScaledDriverRightX() {
        return m_rotLimiter.calculate(-Util.speedScale(m_driverController.getRightXAxis(),
            RobotState.getAutoAlign() ? DriveConstants.AUTO_ALIGN_SPEED_SCALE : DriveConstants.SPEED_SCALE,
            m_driverController.getRightTrigger()));
    }

    public double speedScaledDriverLeftX() {
        return m_xLimiter.calculate(-Util.speedScale(m_driverController.getLeftXAxis(),
            RobotState.getAutoAlign() ? DriveConstants.AUTO_ALIGN_SPEED_SCALE : DriveConstants.SPEED_SCALE,
            m_driverController.getRightTrigger()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get().resetPoseAndRun();
    }
}
