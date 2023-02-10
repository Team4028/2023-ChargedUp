// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.auton.BeakAutonCommand;
import frc.robot.commands.auton.CarsonVPath;
import frc.robot.commands.auton.EpicPath;
import frc.robot.commands.auton.JPath;
import frc.robot.commands.auton.JPath1;
import frc.robot.commands.auton.JPath2;
import frc.robot.commands.auton.NickPath;
import frc.robot.commands.auton.SamPath;
import frc.robot.commands.auton.TestPath;
import frc.robot.commands.auton.TwoPieceAcquirePiece;
import frc.robot.commands.auton.TwoPieceDriveUp;
import frc.robot.commands.auton.TwoPieceScorePiece;
import frc.robot.subsystems.PracticeSwerveDrivetrain;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.utilities.BeakXBoxController;
import frc.robot.utilities.Util;
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
    // Subsystems
    // private final PracticeSwerveDrivetrain m_drive;
    private final SwerveDrivetrain m_drive;
    // private final Flywheel flywheel;
    private final Vision m_vision;

    // Controller
    private final BeakXBoxController m_driverController = new BeakXBoxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<BeakAutonCommand> autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    private final LoggedDashboardNumber flywheelSpeedInput = new LoggedDashboardNumber("Flywheel Speed", 1500.0);

    // Limiters, etc.
    private SlewRateLimiter m_xLimiter = new SlewRateLimiter(4.0);
    private SlewRateLimiter m_yLimiter = new SlewRateLimiter(4.0);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(4.0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_drive = SwerveDrivetrain.getInstance();
        m_vision = Vision.getInstance();

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
        // autoChooser.addOption("Drive With Flywheel", new DriveWithFlywheelAuto(drive, flywheel));

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
    }

    private void initAutonChooser() {
        autoChooser.addDefaultOption("Epic Path", new EpicPath(m_drive));
        autoChooser.addOption("Test Path", new TestPath(m_drive));
        autoChooser.addOption("Carson V Path", new CarsonVPath(m_drive));
        autoChooser.addOption("Sam Path", new SamPath(m_drive));
        autoChooser.addOption("Nick Path", new NickPath(m_drive));
        autoChooser.addOption("j path 1", new JPath1(m_vision, m_drive));
        autoChooser.addOption("j path 2", new JPath2(m_drive));
        autoChooser.addOption("J Path", new JPath(m_drive));
        autoChooser.addOption("Two Piece Drive Up", new TwoPieceDriveUp(m_drive));
        autoChooser.addOption("Two Piece Acquire Piece", new TwoPieceAcquirePiece(m_drive));
        autoChooser.addOption("Two Piece Score Piece", new TwoPieceScorePiece(m_drive));
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
        m_drive.resetOdometry(autoChooser.get().getInitialPose());
        return autoChooser.get();
    }
}
