// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.beaklib.BeakXBoxController;
import frc.lib.beaklib.Util;
import frc.lib.beaklib.drive.swerve.BeakSwerveDrivetrain;
import frc.lib.beaklib.drive.swerve.BeakSwerveDrivetrain.SnapDirection;
import frc.robot.Constants.DriveConstants;
import frc.robot.OneMechanism.GamePieceMode;
import frc.robot.OneMechanism.ScoringPositions;
import frc.robot.commands.arm.CurrentZero;
import frc.robot.commands.auton.Autons;
import frc.robot.commands.auton.BeakAutonCommand;
import frc.robot.commands.chassis.QuadraticAutoBalance;
import frc.robot.commands.chassis.SnapToAngle;
import frc.robot.commands.chassis.XDrive;
import frc.robot.commands.vision.LimelightDrive;
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
    private final Kickstand m_kickstand;
    private final LEDs m_candle;

    // Controller
    private final BeakXBoxController m_driverController = new BeakXBoxController(0);
    private final BeakXBoxController m_operatorController = new BeakXBoxController(1);
    private final BeakXBoxController m_emergencyController = new BeakXBoxController(2);

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
        m_kickstand = Kickstand.getInstance();

        OneMechanism.addSubsystems(m_candle, m_drive, m_frontAprilTagVision, m_lowerArm, m_upperArm, m_wrist);

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
        // ==================
        // DEFAULT COMMANDS
        // ==================
        m_drive.setDefaultCommand(
            new RunCommand(() -> m_drive.drive(
                -speedScaledDriverLeftY(),
                speedScaledDriverLeftX(),
                speedScaledDriverRightX(),
                true),
                m_drive));

        m_gripper.setDefaultCommand(
            new RunCommand(m_gripper::beIdleMode, m_gripper));

        // ================================================
        // DRIVER CONTROLLER - START
        // ZERO DRIVETRAIN
        // ================================================
        m_driverController.start.onTrue(new InstantCommand(m_drive::zero));

        // ================================================
        // DRIVER CONTROLLER - BACK
        // CURRENT-ZERO ROUTINE
        // ================================================
        m_driverController.back.onTrue(m_wrist.runToAngle(ScoringPositions.STOWED.wristAngle)
            .andThen(new CurrentZero(m_upperArm))
            .andThen(new CurrentZero(m_lowerArm))
            .andThen(new WaitCommand(0.5))
            .andThen(m_upperArm.holdArmPosition())
            .andThen(m_lowerArm.holdArmPosition()));

        // ================================================
        // DRIVER CONTROLLER - LT
        // Smart Mode-Sensitive In/Outfeed
        // ================================================
        m_driverController.lt.whileTrue(new ConditionalCommand(
            m_gripper.runMotorIn(),
            m_gripper.modeSensitiveOutfeedCommand(),
            () -> !OneMechanism.getScoreMode())
                .andThen(new InstantCommand(m_gripper::beIdleMode)));

        // ================================================
        // DRIVER CONTROLLER - LB
        // GO TO PURPLE MODE
        // ================================================
        m_driverController.lb.onTrue(new InstantCommand(OneMechanism::becomePurpleMode));

        // ================================================
        // DRIVER CONTROLLER - RB
        // GO TO ORANGE MODE
        // ================================================
        m_driverController.rb.onTrue(new InstantCommand(OneMechanism::becomeOrangeMode));

        // ================================================
        // DRIVER CONTROLLER - A
        // TOGGLE GREEN MODE
        // ================================================
        m_driverController.a.onTrue(new InstantCommand(OneMechanism::toggleClimbMode));

        // ================================================
        // DRIVER CONTROLLER - B
        // AUTO - ALIGN MODE
        // ================================================
        m_driverController.b.onTrue(new InstantCommand(OneMechanism::toggleScoreMode));

        // ================================================
        // DRIVER CONTROLLER - X
        // TOGGLE X - DRIVE
        // ================================================
        m_driverController.x.toggleOnTrue(new XDrive(m_drive));

        // ================================================
        // DRIVER CONTROLLER - Y
        // AUTO-BALANCE
        // ================================================
        m_driverController.y.toggleOnTrue(new QuadraticAutoBalance(m_drive));

        // ================================================
        // DRIVER CONTROLLER - RS
        // CANCEL SNAPS AND GAMEPIECE ALIGN
        // ================================================
        // m_driverController.rs.onTrue()

        // ================================================
        // DRIVER CONTROLLER - DPAD LEFT
        // DECREMENT NODE
        // ================================================
        m_driverController.dpadLeft.onTrue(OneMechanism.decrementNode());

        // ================================================
        // DRIVER CONTROLLER - DPAD RIGHT
        // INCREMENT NODE
        // ================================================
        m_driverController.dpadRight.onTrue(OneMechanism.incrementNode());

        // ================================================
        // DRIVER CONTROLLER - DPAD UP
        // RESET POSE
        // ================================================
        // m_driverController.dpadUp.onTrue(new ResetPoseToVision(m_drive,
        // m_frontAprilTagVision));
        // m_driverController.dpadUp
        // .whileTrue(new RepeatCommand(new FullFieldLocalize(m_drive,
        // m_frontAprilTagVision, m_rearAprilTagVision)));
        m_driverController.dpadUp.toggleOnTrue(new LimelightDrive(m_gripper, m_drive));

        // ================================================
        // DRIVER CONTROLLER - DPAD DOWN
        // RUN TO TARGET NODE POSITION
        // ================================================
        BooleanSupplier nodeInterrupt = () -> Math.abs(speedScaledDriverLeftX()) > 0.1 ||
            Math.abs(speedScaledDriverLeftY()) > 0.1 ||
            Math.abs(speedScaledDriverRightX()) > 0.1;
        // m_driverController.dpadDown.onTrue(OneMechanism.runToNodePosition(nodeInterrupt));
        // m_driverController.dpadDown.toggleOnTrue(new
        // LimelightSquare(m_drive));//.andThen(new LimelightDrive(m_drive)));

        // ================================================
        // OPERATOR CONTROLLER - LB
        // ACQUIRE_SINGLE_SUBSTATION (RAMP)
        // ================================================
        m_operatorController.lb
            .onTrue(OneMechanism.runArms(ScoringPositions.ACQUIRE_SINGLE_SUBSTATION));

        // ================================================
        // OPERATOR CONTROLLER - RB
        // ACQUIRE_DOUBLE_SUBSTATION (WALL)
        // ================================================
        m_operatorController.rb
            .onTrue(new ConditionalCommand(OneMechanism.runArms(ScoringPositions.ACQUIRE_DOUBLE_SUBSTATION_CUBE),
                OneMechanism.runArms(ScoringPositions.ACQUIRE_DOUBLE_SUBSTATION_CONE),
                () -> OneMechanism.getGamePieceMode() == GamePieceMode.PURPLE_CUBE));

        // ================================================
        // OPERATOR CONTROLLER - A
        // SCORE LOW
        // ================================================
        m_operatorController.a.onTrue(new ConditionalCommand(OneMechanism.runArms(ScoringPositions.SCORE_LOW_CUBE),
            OneMechanism.runArms(ScoringPositions.SCORE_LOW_CONE),
            () -> OneMechanism.getGamePieceMode() == GamePieceMode.PURPLE_CUBE));

        // ================================================
        // OPERATOR CONTROLLER - B
        // SCORE MID
        // ================================================
        m_operatorController.b
            .onTrue(new ConditionalCommand(OneMechanism.runArms(ScoringPositions.SCORE_MID_CUBE), // Cubes if Purple
                                                                                                  // Mode
                OneMechanism.runArms(ScoringPositions.SCORE_MID_CONE), // Cones Otherwise
                () -> OneMechanism.getGamePieceMode() == GamePieceMode.PURPLE_CUBE));

        // ================================================
        // OPERATOR CONTROLLER - X
        // STOWED
        // ================================================
        m_operatorController.x
            .onTrue(OneMechanism.runArms(ScoringPositions.STOWED));

        // ================================================
        // OPERATOR CONTROLLER - Y
        // SCORE HIGH
        // ================================================
        m_operatorController.y
            .onTrue(new ConditionalCommand(OneMechanism.runArms(ScoringPositions.SCORE_HIGH_CUBE), // Cubes if Purple
                                                                                                   // Mode
                OneMechanism.runArms(ScoringPositions.SCORE_HIGH_CONE), // Cones Otherwise
                () -> OneMechanism.getGamePieceMode() == GamePieceMode.PURPLE_CUBE));

        // ================================================
        // OPERATOR CONTROLLER - LS
        // ACQUIRE_FLOOR_TIPPED_CONE OR ACQUIRE_FLOOR_CUBE
        // ================================================
        m_operatorController.ls.toggleOnTrue(new LimelightSquare(
            () -> false,
            true,
            () -> -speedScaledDriverLeftY(),
            () -> speedScaledDriverLeftX(),
            m_drive));
        m_operatorController.rs.toggleOnTrue(new LimelightSquare(
            () -> true,
            true,
            () -> -speedScaledDriverLeftY(),
            () -> speedScaledDriverLeftX(),
            m_drive));

        m_operatorController.axisLessThan(1, -0.5)
            .onTrue(OneMechanism.runArms(ScoringPositions.FLOOR_CUBE_SEEK));
        
        m_operatorController.axisGreaterThan(1, 0.5)
            .onTrue(OneMechanism.runArms(ScoringPositions.ACQUIRE_FLOOR_CUBE));

        // ================================================
        // OPERATOR CONTROLLER - RY < -0.5
        // ACQUIRE_FLOOR_UPRIGHT_CONE
        // ================================================
        m_operatorController.axisLessThan(5, -0.5)
            .onTrue(OneMechanism.runArms(ScoringPositions.ACQUIRE_FLOOR_CONE_UPRIGHT));
        
        m_operatorController.axisGreaterThan(5, 0.5)
            .onTrue(OneMechanism.runArms(ScoringPositions.ACQUIRE_FLOOR_CONE_TIPPED));

        // ================================================
        // OPERATOR CONTROLLER - RT
        // SPIT OUT GAMEPIECE
        // ================================================
        m_operatorController.rt.whileTrue(m_gripper.runMotorOut().withTimeout(0.8));

        // ================================================
        // OPERATOR CONTROLLER - LT
        // SOFTLY SPIT OUT GAMEPIECE
        // ================================================
        m_operatorController.lt.whileTrue(m_gripper.runMotorOutSoft().withTimeout(0.8));

        // ================================================
        // OPERATOR CONTROLLER - START
        // ENGAGE KICKSTAND (DOWN)
        // ================================================
        m_operatorController.start.onTrue(m_kickstand.activate());

        // ================================================
        // OPERATOR CONTROLLER - BACK
        // DISENGAGE KICKSTAND (UP)
        // ================================================
        m_operatorController.back.onTrue(m_kickstand.deactivate());

        // ================================================
        // OPERATOR CONTROLLER - DPAD
        // ORTHAGONAL ANGLE HOLDING
        // ================================================
        DoubleSupplier xSupplier = () -> -speedScaledDriverLeftY()
            * m_drive.getPhysics().maxVelocity.getAsMetersPerSecond();
        DoubleSupplier ySupplier = () -> speedScaledDriverLeftX()
            * m_drive.getPhysics().maxVelocity.getAsMetersPerSecond();
        BooleanSupplier angleInterrupt = m_driverController.rs;

        m_operatorController.dpadUp.toggleOnTrue(new SnapToAngle(
            SnapDirection.UP, xSupplier, ySupplier, angleInterrupt, true, m_drive));
        m_operatorController.dpadLeft.toggleOnTrue(new SnapToAngle(
            SnapDirection.LEFT, xSupplier, ySupplier, angleInterrupt, true, m_drive));
        m_operatorController.dpadDown.toggleOnTrue(new SnapToAngle(
            SnapDirection.DOWN, xSupplier, ySupplier, angleInterrupt, true, m_drive));
        m_operatorController.dpadRight.toggleOnTrue(new SnapToAngle(
            SnapDirection.RIGHT, xSupplier, ySupplier, angleInterrupt, true, m_drive));

        // ===============================================
        // EMERGENCY CONTROLLER
        // ===============================================

        // ================================================
        // EMERGENCY CONTROLLER - LOWER ARM MANUAL CONTROLS
        // LSY
        // ================================================
        (m_emergencyController.axisGreaterThan(1, 0.1).or(m_emergencyController.axisLessThan(1, -0.1)))
            .onTrue(new InstantCommand(
                () -> m_lowerArm.runArmVbus(0.3 * -Math.signum(m_emergencyController.getLeftYAxis()))));
        (m_emergencyController.axisGreaterThan(1, 0.1).or(m_emergencyController.axisLessThan(1, -0.1)))
            .onFalse(m_lowerArm.holdArmPosition());

        // ================================================
        // EMERGENCY CONTROLLER - UPPER ARM MANUAL CONTROLS
        // RSX
        // ================================================
        m_emergencyController.axisGreaterThan(4, 0.1).or(m_emergencyController.axisLessThan(4, -0.1))
            .onTrue(new InstantCommand(
                () -> m_upperArm.runArmVbus(0.3 * -Math.signum(m_emergencyController.getRightXAxis()))));
        m_emergencyController.axisGreaterThan(4, 0.1).or(m_emergencyController.axisLessThan(4, -0.1))
            .onFalse(m_upperArm.holdArmPosition());

        // ================================================
        // EMERGENCY CONTROLLER - MOVE THE WRIST UP
        // RT
        // ================================================
        m_emergencyController.rt.whileTrue(m_wrist.runMotor(0.15));

        // ================================================
        // EMERGENCY CONTROLLER - MOVE THE WRIST DOWN
        // LT
        // ================================================
        m_emergencyController.lt.whileTrue(m_wrist.runMotor(-0.15));

        // ================================================
        // EMERGENCY - BUMP LOWER ARM DOWN
        // A
        // ================================================
        m_emergencyController.a.onTrue(m_lowerArm.changePositionCommand(-1.));

        // ================================================
        // EMERGENCY - BUMP LOWER ARM UP
        // Y
        // ================================================
        m_emergencyController.y.onTrue(m_lowerArm.changePositionCommand(1.));

        // ================================================
        // EMERGENCY - BUMP UPPER ARM IN
        // B
        // ================================================
        m_emergencyController.b.onTrue(m_upperArm.changePositionCommand(1.));

        // ================================================
        // EMERGENCY - BUMP UPPER ARM OUT
        // X
        // ================================================
        m_emergencyController.x.onTrue(m_upperArm.changePositionCommand(-1.));

        // ================================================
        // EMERGENCY - BUMP WRIST DOWN
        // LB
        // ================================================
        m_emergencyController.lb.onTrue(m_wrist.changeAngleCommand(-3.));

        // ================================================
        // EMERGENCY - BUMP WRIST UP
        // RB
        // ================================================
        m_emergencyController.rb.onTrue(m_wrist.changeAngleCommand(3.));

        // ================================================
        // EMERGENCY - WaterFall
        // BACK
        // ================================================
        m_emergencyController.back.whileTrue(new InstantCommand(OneMechanism::toggleSlide));

        // ================================================
        // EMERGENCY - SPWC
        // START
        // ================================================
        m_emergencyController.start.onTrue(new InstantCommand(OneMechanism::toggleVictorySpin));

        // ================================================
        // EMERGENCY - FIRE
        // DDOWN
        // ================================================
        m_emergencyController.dpadDown.onTrue(new InstantCommand(OneMechanism::setFireWorkPlz));

        // ================================================
        // EMERGENCY - ACTIVE
        // DRIGHT
        // ================================================
        m_emergencyController.dpadRight.onTrue(new InstantCommand(OneMechanism::setActive));

        // ================================================
        // EMERGENCY - TOGGLE SIGNAL
        // RB
        // ================================================
        // m_emergencyController.rb.onTrue(new
        // InstantCommand(OneMechanism::toggleSnappedMode));
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

        m_autoChooser.addOption("3 Top", m_autons.ThreePiece(PathPosition.Top, false));
        m_autoChooser.addOption("3 Bottom", m_autons.ThreePiece(PathPosition.Bottom, false));

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
            m_autons.OnePieceMobilityBalance());

        m_autoChooser.addOption("Do Nothing", new BeakAutonCommand());

        m_autoChooser.addOption("2 Top Limelight", m_autons.LimelightTwoPiece(PathPosition.Top));
        m_autoChooser.addOption("2 Bottom Limelight", m_autons.LimelightTwoPiece(PathPosition.Bottom));

        m_autoChooser.addOption("3 Top Limelight", m_autons.LimelightThreePiece(PathPosition.Top));
        // m_autoChooser.addOption("3 Bottom Limelight", m_autons.LimelightThreePiece(PathPosition.Bottom));

        // m_autoChooser.addOption("Cool preload sequence", new
        // BeakAutonCommand(m_drive, new Pose2d(),
        // m_autons.coolPreloadScoreSequence()));
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
        return (OneMechanism.getScoreMode() || OneMechanism.getClimbMode()) ? DriveConstants.SLOW_SPEED_SCALE
            : DriveConstants.SPEED_SCALE;
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
