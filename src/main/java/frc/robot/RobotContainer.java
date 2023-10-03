// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.fasterxml.jackson.core.util.RequestPayload;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.beaklib.BeakXBoxController;
import frc.lib.beaklib.Util;
import frc.lib.beaklib.drive.swerve.BeakSwerveDrivetrain;
import frc.robot.Constants.DriveConstants;
import frc.robot.OneMechanism.GamePieceMode;
import frc.robot.OneMechanism.ScoringPositions;
import frc.robot.commands.arm.CurrentZero;
import frc.robot.commands.auton.Autons;
import frc.robot.commands.auton.BeakAutonCommand;
import frc.robot.commands.vision.LimelightSquare;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.arms.LowerArm;
import frc.robot.subsystems.arms.UpperArm;
import frc.robot.subsystems.kickstand.Kickstand;
import frc.robot.subsystems.manipulator.Gripper;
import frc.robot.subsystems.manipulator.Wrist;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.utilities.Trajectories.PathPosition;

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
    private boolean deadmanOn = false;
    // private static final String GAME_PIECE_CAMERA_NAME = "HD_Webcam_C525"; //
    // Very much subject to change.

    private static final Pose3d FRONT_APRILTAG_CAMERA_TO_ROBOT = new Pose3d(Units.inchesToMeters(-3.3125),
        Units.inchesToMeters(5.5625), 0.,
        new Rotation3d(0., Units.degreesToRadians(0.),
            Units.degreesToRadians(180.)));

    private static final Pose3d REAR_APRILTAG_CAMERA_TO_ROBOT = new Pose3d(Units.inchesToMeters(-5.875),
        Units.inchesToMeters(5.625), 0.,
        new Rotation3d(0., Units.degreesToRadians(0.), Units.degreesToRadians(0.)));

    // private static final Pose3d GAME_PIECE_CAMERA_TO_ROBOT = new
    // Pose3d(Units.inchesToMeters(-6),
    // Units.inchesToMeters(-3), 0., new Rotation3d());

    // Subsystems
    private final BeakSwerveDrivetrain m_drive;

    private final Vision m_frontAprilTagVision;
    private final Vision m_rearAprilTagVision;

    private final UpperArm m_upperArm;
    private final LowerArm m_lowerArm;

    private final Gripper m_gripper;
    private final Wrist m_wrist;
    private final LEDs m_candle;

    // Controller
    private final BeakXBoxController m_driverController = new BeakXBoxController(0);
    private final BeakXBoxController m_ourController = new BeakXBoxController(1);

    // Auton stuff
    private final LoggedDashboardChooser<BeakAutonCommand> m_autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    private final Autons m_autons;

    // Limiters, etc.
    private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(4.0);
    private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(4.0);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(4.0);

    private final SlewRateLimiter m_slowXLimiter = new SlewRateLimiter(2.0);
    private final SlewRateLimiter m_slowYLimiter = new SlewRateLimiter(2.0);
    private final SlewRateLimiter m_slowRotLimiter = new SlewRateLimiter(2.0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // m_drive = PracticeSwerveDrivetrain.getInstance();
        // m_drive = PoseEstimatorSwerveDrivetrain.getInstance();

        m_drive = SwerveDrivetrain.getInstance();
        m_frontAprilTagVision = new Vision(FRONT_APRILTAG_CAMERA_NAME, FRONT_APRILTAG_CAMERA_TO_ROBOT);
        m_rearAprilTagVision = new Vision(REAR_APRILTAG_CAMERA_NAME, REAR_APRILTAG_CAMERA_TO_ROBOT);
        m_candle = LEDs.getInstance();

        // m_manipulator = Manipulator.getInstance();
        m_gripper = Gripper.getInstance();
        m_wrist = Wrist.getInstance();

        m_upperArm = UpperArm.getInstance();
        m_lowerArm = LowerArm.getInstance();

        OneMechanism.addSubsystems(m_candle, m_drive, m_lowerArm, m_upperArm, m_wrist);

        m_autons = new Autons(m_drive, m_lowerArm, m_upperArm, m_wrist, m_gripper, m_frontAprilTagVision,
            m_rearAprilTagVision);

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

        // Drive default command
        m_drive.setDefaultCommand(
            makeSupplier(new InstantCommand(() -> m_drive.drive(-speedScaledDriverLeftY(), speedScaledDriverLeftX(),
                speedScaledDriverRightX(), true), m_drive), m_drive).get().repeatedly());

        m_driverController.start.onTrue(new InstantCommand(m_drive::zero));

        // Gripper default command
        m_gripper
            .setDefaultCommand(new InstantCommand(m_gripper::beIdleMode, m_gripper).repeatedly());

        // Zero
        m_driverController.back.onTrue(makeSupplier(m_wrist.runToAngle(ScoringPositions.STOWED.wristAngle)
            .andThen(new CurrentZero(0.65, m_upperArm))
            .andThen(new CurrentZero(0., m_lowerArm))
            .andThen(new WaitCommand(0.5))
            .andThen(m_upperArm.holdArmPosition())
            .andThen(m_lowerArm.holdArmPosition()), m_upperArm, m_lowerArm).get());

        // Infeed
        m_driverController.lt.whileTrue(makeSupplier(m_gripper.runMotorIn().withTimeout(1.)).get());

        // Outfeed
        m_driverController.rt.whileTrue(
            makeSupplier(m_gripper.runMotorOut().withTimeout(1)).get());

        // Purple
        m_driverController.lb.onTrue(new InstantCommand(OneMechanism::becomePurpleMode));

        // Orange
        m_driverController.rb.onTrue(new InstantCommand(OneMechanism::becomeOrangeMode));

        // Stow
        m_driverController.x.onTrue(makeSupplier(OneMechanism.runArms(ScoringPositions.STOWED)).get());

        // Score
        m_driverController.y.onTrue(makeSupplier(OneMechanism.runArms(ScoringPositions.SCORE_MID_CONE)).get());

        // Pickup [line]
        m_driverController.a
            .onTrue(makeSupplier(OneMechanism.runArms(ScoringPositions.ACQUIRE_FLOOR_CONE_UPRIGHT)).get());

        // Seek
        m_driverController.b.onTrue(makeSupplier(new ConditionalCommand(
            new LimelightSquare(false, true,
                () -> -speedScaledDriverLeftY() * m_drive.getPhysics().maxVelocity.getAsMetersPerSecond(),
                () -> speedScaledDriverLeftX() * m_drive.getPhysics().maxVelocity.getAsMetersPerSecond(), m_drive),
            new InstantCommand(() -> {
            }), () -> OneMechanism.getScoringPosition() == ScoringPositions.ACQUIRE_FLOOR_CONE_UPRIGHT)).get());

        // HotFire
        m_driverController.dpadDown.onTrue(new InstantCommand(OneMechanism::setFireWorkPlz));
        // Rainbow
        m_driverController.dpadUp.onTrue(new InstantCommand(OneMechanism::toggleVictorySpin));
        // Slide
        m_driverController.dpadLeft.onTrue(new InstantCommand(OneMechanism::toggleSlide));
        // Normal
        m_driverController.dpadRight.onTrue(new InstantCommand(OneMechanism::setActive));

        //This is for when toddler
        m_ourController.a.onTrue(new InstantCommand(() -> deadmanOn = true))
            .onFalse(new InstantCommand(() -> deadmanOn = false));

        //this is for when led die
        m_ourController.dpadRight.onTrue(new InstantCommand(OneMechanism::setActive));

        //this is for when x.com
        m_ourController.b.onTrue(new InstantCommand(() -> {
        }, m_drive, m_gripper, m_candle, m_lowerArm, m_upperArm, m_wrist));

    }

    /**
     * Wraps a Command in a conditional supplier based on {@link RobotContainer#deadmanOn}
     * @param cmd the command to wrap in a condditional supplier
     * @return the supplier
     */
    private Supplier<Command> makeSupplier(Command cmd, Subsystem... requirements) {
        return () -> deadmanOn ? new InstantCommand(cmd::schedule, requirements) : new InstantCommand(() -> {
        }, requirements);
    }

    private void initAutonChooser() {
        m_autoChooser.addDefaultOption("1.5 Top", m_autons.OnePiece(PathPosition.Top));
        m_autoChooser.addOption("1.5 Bottom", m_autons.OnePiece(PathPosition.Bottom));

        m_autoChooser.addOption("2 Top", m_autons.TwoPiece(PathPosition.Top, false));
        m_autoChooser.addOption("2 Bottom", m_autons.TwoPiece(PathPosition.Bottom, false));

        m_autoChooser.addOption("2 Top Bal", m_autons.TwoPiece(PathPosition.Top, true));
        m_autoChooser.addOption("2 Bottom Bal (OHCL)", m_autons.TwoPiece(PathPosition.Bottom, true));

        m_autoChooser.addOption("2.25 Top", m_autons.TwoQuarterPiece(PathPosition.Top));
        m_autoChooser.addOption("2.25 Bottom (OHCL)", m_autons.TwoQuarterPiece(PathPosition.Bottom));

        m_autoChooser.addOption("2.9 Top", m_autons.ThreePiece(PathPosition.Top, false, false, ""));
        m_autoChooser.addOption("2.9 Bottom", m_autons.ThreePiece(PathPosition.Bottom, false, false, ""));

        m_autoChooser.addOption("3 Top Mid", m_autons.ThreePiece(PathPosition.Top, false, true, "Mid"));

        m_autoChooser.addOption("3 Top Low", m_autons.ThreePiece(PathPosition.Top, false, true, "Low"));

        m_autoChooser.addOption("1 Cube Mid Bal",
            m_autons.OneBalance(PathPosition.Middle, GamePieceMode.PURPLE_CUBE));
        m_autoChooser.addOption("1 Cone Mid Bal",
            m_autons.OneBalance(PathPosition.Middle, GamePieceMode.ORANGE_CONE));

        m_autoChooser.addOption("1 Top Bal",
            new BeakAutonCommand(m_drive, m_autons.Balance(PathPosition.Top, "2").getInitialPose(),
                m_autons.preloadScoreSequence(GamePieceMode.ORANGE_CONE),
                m_autons.Balance(PathPosition.Top, "2")));

        m_autoChooser.addOption("1 Bottom Bal",
            new BeakAutonCommand(m_drive, m_autons.Balance(PathPosition.Bottom, "2").getInitialPose(),
                m_autons.preloadScoreSequence(GamePieceMode.ORANGE_CONE),
                m_autons.Balance(PathPosition.Bottom, "2")));

        m_autoChooser.addOption("1 Mobility Bal",
            m_autons.OnePieceMobilityBalance(true));

        m_autoChooser.addOption("3 Bottom Limelight",
            m_autons.LimelightThreePiece(PathPosition.Bottom, GamePieceMode.PURPLE_CUBE));

        m_autoChooser.addOption("Preload Sequence",
            new BeakAutonCommand(m_autons.preloadScoreSequence(GamePieceMode.ORANGE_CONE)));
        m_autoChooser.addOption("Auton Zero", new BeakAutonCommand(m_autons.autonZero()));
        m_autoChooser.addOption("Do Nothing", new BeakAutonCommand());
    }

    public double speedScaledDriverLeftY() {
        return getCurrentYLimiter().calculate(Util.speedScale(m_driverController.getLeftYAxis(),
            getCurrentSpeedScale(),
            m_driverController.getRightTrigger()));
    }

    public double speedScaledDriverRightX() {
        return getCurrentRotLimiter().calculate(-Util.speedScale(m_driverController.getRightXAxis(),
            getCurrentSpeedScale(),
            m_driverController.getRightTrigger()));
    }

    public double speedScaledDriverLeftX() {
        return getCurrentXLimiter().calculate(-Util.speedScale(m_driverController.getLeftXAxis(),
            getCurrentSpeedScale(),
            m_driverController.getRightTrigger()));
    }

    private double getCurrentSpeedScale() {
        return DriveConstants.SLOW_SPEED_SCALE;
    }

    private SlewRateLimiter getCurrentXLimiter() {
        return (OneMechanism.getScoreMode() || OneMechanism.getClimbMode()) ? m_slowXLimiter : m_xLimiter;
    }

    private SlewRateLimiter getCurrentYLimiter() {
        return (OneMechanism.getScoreMode() || OneMechanism.getClimbMode()) ? m_slowYLimiter : m_yLimiter;
    }

    private SlewRateLimiter getCurrentRotLimiter() {
        return (OneMechanism.getScoreMode() || OneMechanism.getClimbMode()) ? m_slowRotLimiter : m_rotLimiter;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // return m_autons.coolPreloadScoreSequence().andThen(new RunCommand(() ->
        // m_drive.drive(new Chassis)).withTimeout(2.0));
        return m_autoChooser.get().resetPoseAndRun();
    }
}
