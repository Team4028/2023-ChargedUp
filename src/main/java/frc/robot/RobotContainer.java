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
import frc.robot.subsystems.arms.Arm;
import frc.robot.subsystems.arms.LowerArm;
import frc.robot.subsystems.arms.UpperArm;
import frc.robot.Constants;
import frc.robot.subsystems.manipulator.Gripper;
import frc.robot.subsystems.manipulator.Wrist;
import frc.robot.commands.arm.CurrentZero;
import frc.robot.commands.arm.RunArmsToPosition;
import frc.robot.commands.auton.Autons;
import frc.robot.commands.auton.BeakAutonCommand;
import frc.robot.commands.chassis.AddVisionMeasurement;
import frc.robot.commands.chassis.AutoBalance;
import frc.robot.subsystems.swerve.PracticeSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.utilities.Trajectories.PathPosition;
import frc.robot.RobotState.ScoringPositions;
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
    private static final String FRONT_APRILTAG_CAMERA_NAME = "Front_AprilTag_Camera";
    private static final String REAR_APRILTAG_CAMERA_NAME = "Rear_AprilTag_Camera";
    // private static final String GAME_PIECE_CAMERA_NAME = "HD_Webcam_C525"; //
    // Very much subject to change.

    private static final Pose3d FRONT_APRILTAG_CAMERA_TO_ROBOT = new Pose3d(Units.inchesToMeters(-5.),
        Units.inchesToMeters(6.), 0.,
        new Rotation3d(0., Units.degreesToRadians(11.0), Units.degreesToRadians(180.)));

    private static final Pose3d REAR_APRILTAG_CAMERA_TO_ROBOT = new Pose3d(Units.inchesToMeters(-6.),
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

    // private final Manipulator m_manipulator;
    private final Gripper m_gripper;
    private final Wrist m_wrist;

    // Controller
    private final BeakXBoxController m_driverController = new BeakXBoxController(0);
    private final BeakXBoxController m_operatorController = new BeakXBoxController(1);

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
        // m_upperArm = UpperArm.getInstance();
        m_frontAprilTagVision = new Vision(FRONT_APRILTAG_CAMERA_NAME, FRONT_APRILTAG_CAMERA_TO_ROBOT, false);
        m_rearAprilTagVision = new Vision(REAR_APRILTAG_CAMERA_NAME, REAR_APRILTAG_CAMERA_TO_ROBOT, false);

        // m_manipulator = Manipulator.getInstance();
        m_gripper = Gripper.getInstance();
        m_wrist = Wrist.getInstance();

        m_upperArm = UpperArm.getInstance();
        m_lowerArm = LowerArm.getInstance();

        m_autons = new Autons(m_drive, m_lowerArm, m_frontAprilTagVision, m_rearAprilTagVision);

        RobotState.addSubsystem(null);

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
        //zeros the drivetrain
        m_driverController.start.onTrue(new InstantCommand(m_drive::zero));

        
        //Arm positions
        m_driverController.a.onTrue(new RunArmsToPosition(ScoringPositions.INTERMEDIATE_LOW, m_lowerArm, m_upperArm, m_wrist).andThen(new RunArmsToPosition(ScoringPositions.STOWED, m_lowerArm, m_upperArm, m_wrist)));
        m_driverController.b.onTrue(new RunArmsToPosition(ScoringPositions.INTERMEDIATE_LOW, m_lowerArm, m_upperArm, m_wrist).andThen(new RunArmsToPosition(ScoringPositions.ACQUIRE_FLOOR_TIPPED_CONE, m_lowerArm, m_upperArm, m_wrist)));
        m_driverController.y.onTrue(new RunArmsToPosition(ScoringPositions.ACQUIRE_FLOOR_UPRIGHT_CONE, m_lowerArm, m_upperArm, m_wrist));
        m_driverController.x.onTrue(new RunArmsToPosition(ScoringPositions.SCORE_MID,
            m_lowerArm, m_upperArm, m_wrist));

        m_driverController.rb.onTrue(new AutoBalance(m_drive));
        m_driverController.lb.onTrue(new AddVisionMeasurement(m_drive, m_frontAprilTagVision));

        //Zeros the arms
        m_driverController.back.onTrue(m_wrist.runToAngle(ScoringPositions.STOWED.wristAngle)
            .andThen(new CurrentZero(m_upperArm)
            .andThen(new CurrentZero(m_lowerArm))));
            // .andThen(new WaitCommand(3.0))))
            // .andThen(new RunArmsToPosition(ScoringPositions.STOWED, m_lowerArm, m_upperArm, m_wrist)));
        //.andThen(new WaitCommand(1.0)).andThen(new RunArmsToPosition(Arm.ArmPositions.RETRACTED, Wrist.WristPositions.STOW, m_lowerArm, m_upperArm, m_wrist)));
       
        // infeed
        m_operatorController.rb.onTrue(m_gripper.runMotorIn());
        m_operatorController.rb.onFalse(m_gripper.stopMotor());
        m_operatorController.lb.onTrue(m_gripper.runMotorOut());
        m_operatorController.lb.onFalse(m_gripper.stopMotor());

        // wrist
        m_operatorController.rt.onTrue(m_wrist.runMotorUp());
        m_operatorController.rt.onFalse(m_wrist.stopMotor());
        m_operatorController.lt.onTrue(m_wrist.runMotorDown());
        m_operatorController.lt.onFalse(m_wrist.stopMotor());

        // mode
        // m_operatorController.x.onTrue(new InstantCommand(() -> RobotState.modeCube()));
        // m_operatorController.y.onTrue(new InstantCommand(() -> RobotState.modeCone()));
        // m_operatorController.b.onTrue(new InstantCommand(() -> RobotState.modeBlank()));

        //raw vbus runs the arms
        m_operatorController.a.onTrue(new InstantCommand(() -> m_upperArm.runArm(0.15)));
        m_operatorController.a.onFalse(new InstantCommand(() -> m_upperArm.runArm(0)));
        m_operatorController.b.onTrue(new InstantCommand(() -> m_upperArm.runArm(-0.15)));
        m_operatorController.b.onFalse(new InstantCommand(() -> m_upperArm.runArm(0.0)));
        m_operatorController.x.onTrue(new InstantCommand(() -> m_lowerArm.runArm(0.15)));
        m_operatorController.x.onFalse(new InstantCommand(() -> m_lowerArm.runArm(0.0)));
        m_operatorController.y.onTrue(new InstantCommand(() -> m_lowerArm.runArm(-0.15)));
        m_operatorController.y.onFalse(new InstantCommand(() -> m_lowerArm.runArm(0.0)));
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
