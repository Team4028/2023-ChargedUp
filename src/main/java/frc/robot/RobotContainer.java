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
import frc.lib.beaklib.BeakXBoxController;
import frc.lib.beaklib.Util;
import frc.lib.beaklib.drive.swerve.BeakSwerveDrivetrain;
import frc.lib.beaklib.drive.swerve.BeakSwerveDrivetrain.SnapDirection;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.arms.LowerArm;
import frc.robot.subsystems.arms.UpperArm;
import frc.robot.subsystems.kickstand.Kickstand;
import frc.robot.subsystems.manipulator.Gripper;
import frc.robot.subsystems.manipulator.Wrist;
import frc.robot.commands.arm.CurrentZero;
import frc.robot.commands.auton.Autons;
import frc.robot.commands.auton.BeakAutonCommand;
import frc.robot.commands.chassis.FullFieldLocalize;
import frc.robot.commands.chassis.KeepAngle;
import frc.robot.commands.chassis.QuadraticAutoBalance;
import frc.robot.commands.chassis.XDrive;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Vision;
import frc.robot.utilities.Trajectories.PathPosition;
import frc.robot.OneMechanism.GamePieceMode;
import frc.robot.OneMechanism.ScoringPositions;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

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
    private final LoggedDashboardChooser<BeakAutonCommand> autoChooser = new LoggedDashboardChooser<>("Auto Choices");
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

        if (Constants.PRACTICE_CHASSIS) {
            // m_manipulator = Manipulator.getInstance();
            m_gripper = Gripper.getInstance();
            m_wrist = Wrist.getInstance();

            m_upperArm = UpperArm.getInstance();
            m_lowerArm = LowerArm.getInstance();
            m_kickstand = Kickstand.getInstance();
        } else {
            m_gripper = null;
            m_wrist = null;

            m_upperArm = null;
            m_lowerArm = null;
        }

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
            new RunCommand(() -> m_gripper.beIdleMode(), m_gripper));

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
            () -> !OneMechanism.getAutoAlignMode())
                .andThen(new InstantCommand(() -> m_gripper.beIdleMode())));

        // ================================================
        // DRIVER CONTROLLER - LB
        // GO TO PURPLE MODE
        // ================================================
        m_driverController.lb.onTrue(new InstantCommand(() -> OneMechanism.becomePurpleMode()));

        // ================================================
        // DRIVER CONTROLLER - RB
        // GO TO ORANGE MODE
        // ================================================
        m_driverController.rb.onTrue(new InstantCommand(() -> OneMechanism.becomeOrangeMode()));

        // ================================================
        // DRIVER CONTROLLER - A
        // TOGGLE GREEN MODE
        // ================================================
        m_driverController.a.onTrue(new InstantCommand(() -> OneMechanism.toggleGreen()));

        // ================================================
        // DRIVER CONTROLLER - B
        // AUTO - ALIGN MODE
        // ================================================
        m_driverController.b.onTrue(new InstantCommand(() -> OneMechanism.toggleAutoAlign()));

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
        m_driverController.dpadUp
            .whileTrue(new RepeatCommand(new FullFieldLocalize(m_drive, m_frontAprilTagVision, m_rearAprilTagVision)));

        // ================================================
        // DRIVER CONTROLLER - DPAD DOWN
        // RUN TO TARGET NODE POSITION
        // ================================================
        BooleanSupplier nodeInterrupt = () -> Math.abs(speedScaledDriverLeftX()) > 0.1 ||
            Math.abs(speedScaledDriverLeftY()) > 0.1 ||
            Math.abs(speedScaledDriverRightX()) > 0.1;
        m_driverController.dpadDown.onTrue(OneMechanism.runToNodePosition(nodeInterrupt));

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
        m_operatorController.ls
            .onTrue(new ConditionalCommand(OneMechanism.runArms(ScoringPositions.ACQUIRE_FLOOR_CUBE), // Cubes if Purple
                                                                                                      // Mode
                OneMechanism.runArms(ScoringPositions.ACQUIRE_FLOOR_CONE_TIPPED), // Cones Otherwise
                () -> OneMechanism.getGamePieceMode() == GamePieceMode.PURPLE_CUBE));

        // ================================================
        // OPERATOR CONTROLLER - RS
        // ACQUIRE_FLOOR_UPRIGHT_CONE
        // ================================================
        m_operatorController.rs
            .onTrue(new ConditionalCommand(OneMechanism.runArms(ScoringPositions.ACQUIRE_FLOOR_CONE_UPRIGHT),
                new InstantCommand(() -> {
                }),
                () -> OneMechanism.getGamePieceMode() == GamePieceMode.ORANGE_CONE));

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

        m_operatorController.dpadUp.toggleOnTrue(new KeepAngle(
            SnapDirection.UP, xSupplier, ySupplier, angleInterrupt, m_drive));
        m_operatorController.dpadLeft.toggleOnTrue(new KeepAngle(
            SnapDirection.LEFT, xSupplier, ySupplier, angleInterrupt, m_drive));
        m_operatorController.dpadDown.toggleOnTrue(new KeepAngle(
            SnapDirection.DOWN, xSupplier, ySupplier, angleInterrupt, m_drive));
        m_operatorController.dpadRight.toggleOnTrue(new KeepAngle(
            SnapDirection.RIGHT, xSupplier, ySupplier, angleInterrupt, m_drive));

        // ===============================================
        // EMERGENCY CONTROLLER
        // ===============================================

        // ================================================
        // EMERGENCY CONTROLLER - LOWER ARM MANUAL CONTROLS
        // LSY
        // ================================================
        (m_emergencyController.axisGreaterThan(1, 0.1).or(m_emergencyController.axisLessThan(1, -0.1)))
            .onTrue(new InstantCommand(() -> m_lowerArm.runArmVbus(0.3 * -Math.signum(m_emergencyController.getLeftYAxis()))));
        (m_emergencyController.axisGreaterThan(1, 0.1).or(m_emergencyController.axisLessThan(1, -0.1)))
            .onFalse(m_lowerArm.holdArmPosition());

        // ================================================
        // EMERGENCY CONTROLLER - UPPER ARM MANUAL CONTROLS
        // RSX
        // ================================================
        m_emergencyController.axisGreaterThan(4, 0.1).or(m_emergencyController.axisLessThan(4, -0.1))
            .onTrue(new InstantCommand(() -> m_upperArm.runArmVbus(0.3 * -Math.signum(m_emergencyController.getRightXAxis()))));
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
        // EMERGENCY - WaterFall
        // B
        // ================================================
        m_emergencyController.b.whileTrue(new InstantCommand(() -> OneMechanism.toggleSlide()));

        // ================================================
        // EMERGENCY - SPWC
        // START
        // ================================================
        m_emergencyController.start.onTrue(new InstantCommand(() -> OneMechanism.toggleVictorySpin()));

        // ================================================
        // EMERGENCY - FIRE
        // DDOWN
        // ================================================
        m_emergencyController.dpadDown.onTrue(new InstantCommand(() -> OneMechanism.setFireWorkPlz()));

        // ================================================
        // EMERGENCY - ACTIVE
        // DRIGHT
        // ================================================
        m_emergencyController.dpadRight.onTrue(new InstantCommand(() -> OneMechanism.setActive()));

        // ================================================
        // EMERGENCY - TOGGLE SIGNAL
        // RB
        // ================================================
        m_emergencyController.rb.onTrue(new InstantCommand(() -> OneMechanism.toggleBlueMode()));
    }

    private void initAutonChooser() {
        autoChooser.addDefaultOption("1.5 Piece Top", m_autons.OnePiece(PathPosition.Top));
        autoChooser.addOption("1.5 Piece Bottom", m_autons.OnePiece(PathPosition.Bottom));

        autoChooser.addOption("2 Piece Top", m_autons.TwoPiece(PathPosition.Top, false));
        autoChooser.addOption("2 Piece Bottom", m_autons.TwoPiece(PathPosition.Bottom, false));

        autoChooser.addOption("2 Piece Top Balance", m_autons.TwoPiece(PathPosition.Top, true));
        autoChooser.addOption("2 Piece Bottom Balance", m_autons.TwoPiece(PathPosition.Bottom, true));

        autoChooser.addOption("3 Piece Top", m_autons.ThreePiece(PathPosition.Top, false));
        autoChooser.addOption("3 Piece Bottom", m_autons.ThreePiece(PathPosition.Bottom, false));
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
        return (OneMechanism.getAutoAlignMode() || OneMechanism.getClimbMode()) ? DriveConstants.SLOW_SPEED_SCALE
            : DriveConstants.SPEED_SCALE;
    }

    private SlewRateLimiter getCurrentXLimiter() {
        return (OneMechanism.getAutoAlignMode() || OneMechanism.getClimbMode()) ? m_slowXLimiter : m_xLimiter;
    }

    private SlewRateLimiter getCurrentYLimiter() {
        return (OneMechanism.getAutoAlignMode() || OneMechanism.getClimbMode()) ? m_slowYLimiter : m_yLimiter;
    }

    private SlewRateLimiter getCurrentRotLimiter() {
        return (OneMechanism.getAutoAlignMode() || OneMechanism.getClimbMode()) ? m_slowRotLimiter : m_rotLimiter;
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
