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
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Autons;
import frc.robot.commands.CurrentZero;
import frc.robot.commands.RunArm;
import frc.robot.commands.RunArmsToPosition;
import frc.robot.commands.Autons.TwoPiecePositions;
import frc.robot.commands.auton.BeakAutonCommand;
import frc.robot.subsystems.arms.Arm;
import frc.robot.subsystems.arms.LowerArm;
import frc.robot.subsystems.arms.UpperArm;
import frc.robot.subsystems.PoseEstimatorSwerveDrivetrain;
import frc.robot.subsystems.PracticeSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.utilities.BeakXBoxController;
import frc.robot.utilities.Util;
import frc.robot.utilities.drive.swerve.BeakSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
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
    private static final String FRONT_APRILTAG_CAMERA_NAME = "Front_AprilTag_Camera";
    private static final String REAR_APRILTAG_CAMERA_NAME = "Rear_AprilTag_Camera";
    // private static final String GAME_PIECE_CAMERA_NAME = "HD_Webcam_C525"; //
    // Very much subject to change.

    private static final Pose3d FRONT_APRILTAG_CAMERA_TO_ROBOT = new Pose3d(Units.inchesToMeters(-5.),
        Units.inchesToMeters(6.), 0.,
        new Rotation3d(0., Units.degreesToRadians(14.), Units.degreesToRadians(0.)));

    private static final Pose3d REAR_APRILTAG_CAMERA_TO_ROBOT = new Pose3d(Units.inchesToMeters(-6.),
        Units.inchesToMeters(6.), 0.,
        new Rotation3d(0., Units.degreesToRadians(14.), Units.degreesToRadians(0.)));
    // private static final Pose3d GAME_PIECE_CAMERA_TO_ROBOT = new
    // Pose3d(Units.inchesToMeters(12.), 0., 0., new Rotation3d());

    // Subsystems
    private final BeakSwerveDrivetrain m_drive;
    private final Vision m_frontAprilTagVision;
    private final Vision m_rearAprilTagVision;
    // private final UpperArm m_upperArm;
    private final LowerArm m_lowerArm;

    // Controller
    private final BeakXBoxController m_driverController = new BeakXBoxController(0);
    // private final BeakXBoxController m_operatorController = new
    // BeakXBoxController(1);

    // Auton stuff
    private final LoggedDashboardChooser<BeakAutonCommand> autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    private final Autons m_autons;

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
        // m_upperArm = UpperArm.getInstance();
        m_lowerArm = LowerArm.getInstance();
        m_frontAprilTagVision = new Vision(FRONT_APRILTAG_CAMERA_NAME, FRONT_APRILTAG_CAMERA_TO_ROBOT, true);
        m_rearAprilTagVision = new Vision(REAR_APRILTAG_CAMERA_NAME, REAR_APRILTAG_CAMERA_TO_ROBOT, true);
        // m_gamePieceVision = new Vision(GAME_PIECE_CAMERA_NAME,
        // GAME_PIECE_CAMERA_TO_ROBOT, false);

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

        m_driverController.start.onTrue(new InstantCommand(m_drive::zero));
        m_driverController.a.onTrue(new InstantCommand(() -> m_lowerArm.runToPosition(0.)));
        m_driverController.b.onTrue(new RunArm(45., m_lowerArm));
        m_driverController.x.onTrue(new RunArm(12., m_lowerArm));
        m_driverController.y.onTrue(new RunArm(14., m_lowerArm));
        // m_driverController.a.onTrue(new RunArmsToPosition(Arm.ArmPositions.RETRACTED,
        // m_lowerArm, m_upperArm));
        // m_driverController.b.onTrue(new
        // RunArmsToPosition(Arm.ArmPositions.ACQUIRE_FLOOR, m_lowerArm, m_upperArm));
        // m_driverController.x.onTrue(new RunArmsToPosition(Arm.ArmPositions.SCORE_MID,
        // m_lowerArm, m_upperArm));
        // m_driverController.y.onTrue(new
        // RunArmsToPosition(Arm.ArmPositions.SCORE_HIGH, m_lowerArm, m_upperArm));

        // m_driverController.lb.whileTrue(new InstantCommand(() ->
        // m_upperArm.runArm(-0.4)));
        // m_driverController.lb.onFalse(new InstantCommand(() ->
        // m_upperArm.runArm(0.0)));

        // m_driverController.rb.whileTrue(new InstantCommand(() ->
        // m_upperArm.runArm(0.4)));
        // m_driverController.rb.onFalse(new InstantCommand(() ->
        // m_upperArm.runArm(0.0)));

        // m_driverController.lt.whileTrue(new InstantCommand(() ->
        // m_lowerArm.runArm(-0.4)));
        // m_driverController.lt.onFalse(new InstantCommand(() ->
        // m_lowerArm.runArm(0.0)));

        // TODO: Fix this doodoo
        // m_driverController.rt.whileTrue(new InstantCommand(() ->
        // m_lowerArm.runArm(0.4)));
        // m_driverController.rt.onFalse(new InstantCommand(() ->
        // m_lowerArm.runArm(0.0)));

        // m_driverController.dpadUp.onTrue(m_drive.generatePath(() ->
        // m_gamePieceVision.getTargetPose(m_drive.getPoseMeters(),
        // new Transform3d(new Translation3d(Units.inchesToMeters(5.),
        // Units.inchesToMeters(-0.), 0.),
        // new Rotation3d()))));
        // m_driverController.dpadDown.onTrue(new InstantCommand(() ->
        // m_gamePieceVision.togglePipeline()));

        // m_driverController.back.onTrue(new CurrentZero(m_upperArm).andThen(new
        // CurrentZero(m_lowerArm)));4
        m_driverController.back.onTrue(new CurrentZero(m_lowerArm));
    }

    private void initAutonChooser() {
        autoChooser.addDefaultOption("j path 1", m_autons.JPath1());

        // autoChooser.addOption("j path 2", new JPath2(m_drive));
        // autoChooser.addOption("J Path", new JPath(m_drive));

        autoChooser.addOption("Two Piece Top", m_autons.TwoPiece(TwoPiecePositions.TOP));
        autoChooser.addOption("Two Piece Top Acquire", m_autons.TwoPieceAcquire(TwoPiecePositions.TOP));
        autoChooser.addOption("Two Piece Top Score", m_autons.TwoPieceScore(TwoPiecePositions.TOP));
        autoChooser.addOption("Two Piece Bottom", m_autons.TwoPiece(TwoPiecePositions.BOTTOM));
        autoChooser.addOption("Two Piece Bottom Acquire", m_autons.TwoPieceAcquire(TwoPiecePositions.BOTTOM));
        autoChooser.addOption("Two Piece Bottom Score", m_autons.TwoPieceScore(TwoPiecePositions.BOTTOM));
    }

    public double speedScaledDriverLeftY() {
        return m_yLimiter.calculate(Util.speedScale(m_driverController.getLeftYAxis(),
            DriveConstants.SPEED_SCALE,
            m_driverController.getRightTrigger()));
    }

    public double speedScaledDriverRightX() {
        return m_rotLimiter.calculate(-Util.speedScale(m_driverController.getRightXAxis(),
            DriveConstants.SPEED_SCALE,
            m_driverController.getRightTrigger()));
    }

    public double speedScaledDriverLeftX() {
        return m_xLimiter.calculate(-Util.speedScale(m_driverController.getLeftXAxis(),
            DriveConstants.SPEED_SCALE,
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
