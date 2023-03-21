// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.arms.LowerArm;
import frc.robot.subsystems.arms.UpperArm;
import frc.robot.subsystems.kickstand.Kickstand;
import frc.robot.subsystems.manipulator.Gripper;
import frc.robot.subsystems.manipulator.Wrist;
import frc.robot.commands.arm.CurrentZero;
import frc.robot.commands.arm.RunArmsToPosition;
import frc.robot.commands.arm.RunArmsSafely;
import frc.robot.commands.auton.Autons;
import frc.robot.commands.auton.BeakAutonCommand;
import frc.robot.commands.chassis.AutoBalance;
import frc.robot.commands.chassis.ResetPoseToVision;
import frc.robot.commands.kickstand.AcitvateKickstand;
import frc.robot.commands.kickstand.DeactivateKickstand;
import frc.robot.subsystems.swerve.PracticeSwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Vision;
import frc.robot.utilities.Trajectories.PathPosition;
import frc.robot.OneMechanism.GamePieceMode;
import frc.robot.OneMechanism.ScoringPositions;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    private static final Pose3d FRONT_APRILTAG_CAMERA_TO_ROBOT = Constants.PRACTICE_CHASSIS
        ? new Pose3d(Units.inchesToMeters(5.),
            Units.inchesToMeters(6.), 0.,
            new Rotation3d(0., Units.degreesToRadians(11.0),
                Units.degreesToRadians(0.)))
        : new Pose3d(Units.inchesToMeters(-2.),
            Units.inchesToMeters(-2.), 0.,
            new Rotation3d(0., Units.degreesToRadians(11.0), Units.degreesToRadians(-8.)));

    private static final Pose3d REAR_APRILTAG_CAMERA_TO_ROBOT = new Pose3d(Units.inchesToMeters(6.),
        Units.inchesToMeters(6.), 0.,
        new Rotation3d(0., Units.degreesToRadians(16.0), Units.degreesToRadians(180.)));
    // private static final Pose3d GAME_PIECE_CAMERA_TO_ROBOT = new
    // Pose3d(Units.inchesToMeters(12.), 0., 0., new Rotation3d());

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
        // m_drive = PracticeSwerveDrivetrain.getInstance();
        // m_drive = PoseEstimatorSwerveDrivetrain.getInstance();

        m_drive = SwerveDrivetrain.getInstance();
        m_frontAprilTagVision = new Vision(FRONT_APRILTAG_CAMERA_NAME, FRONT_APRILTAG_CAMERA_TO_ROBOT, false);
        m_rearAprilTagVision = new Vision(REAR_APRILTAG_CAMERA_NAME, REAR_APRILTAG_CAMERA_TO_ROBOT, false);
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
        // TODO - Actual Competition control will be Back to toggle GamePiece Mode.
        m_driverController.back.onTrue(m_wrist.runToAngle(ScoringPositions.STOWED.wristAngle)
            .andThen(new CurrentZero(m_upperArm))
            .andThen(new CurrentZero(m_lowerArm))
            .andThen(new WaitCommand(0.5))
            .andThen(m_upperArm.holdArmPosition())
            .andThen(m_lowerArm.holdArmPosition()));
        // We cannot do .andThen runArmsToPosition because the encoder zeroes are not
        // read properly
        // by the SequentialCommandGroup.

        // ================================================
        // DRIVER CONTROLLER - LT
        // RUN GRIPPER IN (WITH SMART HOLDING)
        // ================================================
        m_driverController.lt.whileTrue(m_gripper.runMotorIn().until(m_gripper.atCurrentThresholdSupplier())
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

        // ================================================
        // DRIVER CONTROLLER - Y
        // AUTO-BALANCE
        // ================================================
        m_driverController.y.onTrue(new AutoBalance(m_drive));

        // ================================================
        // DRIVER CONTROLLER - DLEFT
        // DECREMENT NODE
        // ================================================
        m_driverController.dpadLeft.onTrue(new InstantCommand(() -> OneMechanism.decrementNode()));
        // ================================================
        // DRIVER CONTROLLER - DRIGHT
        // INCREMENT NODE
        // ================================================
        m_driverController.dpadRight.onTrue(new InstantCommand(() -> OneMechanism.incrementNode()));

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
        // OPERATOR CONTROLLER - X
        // STOWED
        // ================================================
        m_operatorController.x
            .onTrue(OneMechanism.runArms(ScoringPositions.STOWED));

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
        // ACQUIRE_FLOOR_TIPPED_CONE OR ACQUIRE_FLOOR_CUBES
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
        m_operatorController.rt.onTrue(m_gripper.runMotorOut().withTimeout(0.8));
        m_operatorController.rt.onFalse(m_gripper.stopMotor());

        // ================================================
        // OPERATOR CONTROLLER - LT
        // SOFTLY SPIT OUT GAMEPIECE
        // ================================================
        m_operatorController.lt.onTrue(m_gripper.runMotorOutSoft().withTimeout(0.8));
        m_operatorController.lt.onFalse(m_gripper.stopMotor());

        // ================================================
        // OPERATOR CONTROLLER - START
        // ENGAGE KICKSTAND (DOWN)
        // ================================================
        m_operatorController.start.onTrue(new AcitvateKickstand(m_kickstand));

        // ================================================
        // OPERATOR CONTROLLER - BACK
        // DISENGAGE KICKSTAND (UP)
        // ================================================
        m_operatorController.back.onTrue(new DeactivateKickstand(m_kickstand));

        // ================================================
        // EMERGENCY CONTROLLER - LOWER ARM MANUAL CONTROLS
        // LSY
        // ================================================
        m_emergencyController.axisGreaterThan(1, 0.1)
            .onTrue(new InstantCommand(() -> m_lowerArm.runArmVbus(-0.5 * m_emergencyController.getLeftYAxis())));
        m_emergencyController.axisGreaterThan(1, 0.0)
            .onFalse(new ConditionalCommand(
                new InstantCommand(() -> m_lowerArm.runArmVbus(0.3 * Math.abs(m_emergencyController.getLeftYAxis()))),
                m_lowerArm.holdArmPosition(),
                () -> m_emergencyController.axisLessThan(1, -0.1).getAsBoolean()));

        // ================================================
        // EMERGENCY CONTROLLER - UPPER ARM MANUAL CONTROLS
        // RSX
        // ================================================
        m_emergencyController.axisGreaterThan(4, 0.1)
            .onTrue(new InstantCommand(() -> m_upperArm.runArmVbus(-0.5 * m_emergencyController.getRightXAxis())));
        m_emergencyController.axisGreaterThan(4, 0.0)
            .onFalse(new ConditionalCommand(
                new InstantCommand(() -> m_upperArm.runArmVbus(0.5 * Math.abs(m_emergencyController.getRightXAxis()))),
                m_upperArm.holdArmPosition(),
                () -> m_emergencyController.axisLessThan(4, -0.1).getAsBoolean()));

        // ================================================
        // EMERGENCY CONTROLLER - MOVE THE WRIST UP
        // RT
        // ================================================
        m_emergencyController.rt.onTrue(m_wrist.runMotorUp());
        m_emergencyController.rt.onFalse(m_wrist.holdWristAngle());

        // ================================================
        // EMERGENCY CONTROLLER - MOVE THE WRIST DOWN
        // LT
        // ================================================
        m_emergencyController.lt.onTrue(m_wrist.runMotorDown());
        m_emergencyController.lt.onFalse(m_wrist.holdWristAngle());

        // ================================================
        // EMERGENCY - RESET POSE
        // A
        // ================================================
        // m_emergencyController.a.onTrue(new ResetPoseToVision(m_drive,
        // m_frontAprilTagVision));

        // ================================================
        // EMERGENCY CONTROLLER - ACTIVE
        // A
        // ================================================
        m_emergencyController.a.onTrue(new InstantCommand(() -> OneMechanism.setActive()));

        // ================================================
        // EMERGENCY CONTROLLER - IDLE
        // B
        // ================================================
        m_emergencyController.b.onTrue(new InstantCommand(() -> OneMechanism.setIdle()));

        // ================================================
        // EMERGENCY - EXPERIMENTAL - FIRE
        // X
        // ================================================
        m_emergencyController.x.onTrue(new InstantCommand(() -> OneMechanism.setFire()));

        // ================================================
        // EMERGENCY CONTROLLER - VICTORY SPIN
        // Y
        // ================================================
        m_emergencyController.y.onTrue(new InstantCommand(() -> OneMechanism.toggleVictorySpin()));

        // ================================================
        // EMERGENCY CONTROLLER - PROBABLY BAD - IDLE V2
        // LS
        // ================================================
        m_emergencyController.ls.onTrue(new InstantCommand(() -> OneMechanism.setIdleV2()));

        // ================================================
        // EMERGENCY CONTROLLER - EXPERIMENTAL - TOGGLE THROW ON GROUND SLIDE SIGNAL
        // RS
        // ================================================
        m_emergencyController.rs.onTrue(new InstantCommand(() -> OneMechanism.toggleThrowOnGround()));

        // ================================================
        // EMERGENCY CONTROLLER - EXPERIMENTAL - IDLEV3
        // RB
        // ================================================
        m_emergencyController.rb.onTrue(new InstantCommand(() -> OneMechanism.setIdleV3()));
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
            OneMechanism.getAutoAlignMode() ? DriveConstants.AUTO_ALIGN_SPEED_SCALE : DriveConstants.SPEED_SCALE,
            m_driverController.getRightTrigger()));
    }

    public double speedScaledDriverRightX() {
        return m_rotLimiter.calculate(-Util.speedScale(m_driverController.getRightXAxis(),
            OneMechanism.getAutoAlignMode() ? DriveConstants.AUTO_ALIGN_SPEED_SCALE : DriveConstants.SPEED_SCALE,
            m_driverController.getRightTrigger()));
    }

    public double speedScaledDriverLeftX() {
        return m_xLimiter.calculate(-Util.speedScale(m_driverController.getLeftXAxis(),
            OneMechanism.getAutoAlignMode() ? DriveConstants.AUTO_ALIGN_SPEED_SCALE : DriveConstants.SPEED_SCALE,
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
