// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
//import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.CurrentZero;
import frc.robot.commands.RunArmsToPosition;
import frc.robot.subsystems.PracticeSwerveDrivetrain;
import frc.robot.subsystems.arms.Arm;
import frc.robot.subsystems.arms.LowerArm;
import frc.robot.subsystems.arms.UpperArm;
// import frc.robot.subsystems.flywheel.Flywheel;
// import frc.robot.subsystems.flywheel.FlywheelIO;
// import frc.robot.subsystems.flywheel.FlywheelIOSim;
// import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
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
    private final PracticeSwerveDrivetrain m_drive;
    // private final Flywheel flywheel;
    private final UpperArm m_upperArm;
    private final LowerArm m_lowerArm;

    // Controller
    private final BeakXBoxController m_driverController = new BeakXBoxController(0);
    //private final BeakXBoxController m_operatorController = new BeakXBoxController(1);

    // Dashboard inputs
    // TODO: Convert to BeakAutonCommand
    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    // private final LoggedDashboardNumber flywheelSpeedInput = new LoggedDashboardNumber("Flywheel Speed", 1500.0);

    // Limiters, etc.
    private SlewRateLimiter m_xLimiter = new SlewRateLimiter(4.0);
    private SlewRateLimiter m_yLimiter = new SlewRateLimiter(4.0);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(4.0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_drive = PracticeSwerveDrivetrain.getInstance();
        m_upperArm = UpperArm.getInstance();
        m_lowerArm=LowerArm.getInstance();
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
        autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
        // autoChooser.addOption("Spin", new SpinAuto(drive));
        // autoChooser.addOption("Drive With Flywheel", new DriveWithFlywheelAuto(drive, flywheel));

        // Configure the button bindings
        configureButtonBindings();
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
        m_driverController.a.onTrue(new RunArmsToPosition(Arm.ArmPositions.RETRACTED, m_lowerArm, m_upperArm));
        m_driverController.b.onTrue(new RunArmsToPosition(Arm.ArmPositions.THIRTY, m_lowerArm, m_upperArm));
        m_driverController.x.onTrue(new RunArmsToPosition(Arm.ArmPositions.SIXTY, m_lowerArm, m_upperArm));
        m_driverController.y.onTrue(new RunArmsToPosition(Arm.ArmPositions.NINETY, m_lowerArm, m_upperArm));

        m_driverController.lb.whileTrue(new InstantCommand(()->m_upperArm.runArm(0.7)));
        m_driverController.lb.onFalse(new InstantCommand(()->m_upperArm.runArm(0.0)));

        m_driverController.rb.whileTrue(new InstantCommand(()->m_upperArm.runArm(-0.7)));
        m_driverController.rb.onFalse(new InstantCommand(()->m_upperArm.runArm(0.0)));

        m_driverController.lt.whileTrue(new InstantCommand(()->m_lowerArm.runArm(-0.7)));
        m_driverController.lt.onFalse(new InstantCommand(()->m_lowerArm.runArm(0.0)));

        m_driverController.rt.whileTrue(new InstantCommand(()->m_lowerArm.runArm(0.7)));
        m_driverController.rt.onFalse(new InstantCommand(()->m_lowerArm.runArm(0.0)));

        m_driverController.back.whileTrue(new CurrentZero(m_upperArm).alongWith(new CurrentZero(m_lowerArm)));

        // m_operatorController.a.whileTrue(new InstantCommand(m_arm2::armTen));
        // m_operatorController.b.whileTrue(new InstantCommand(m_arm2::armThirty));
        // m_operatorController.x.whileTrue(new InstantCommand(m_arm2::armSixty));
        // m_operatorController.y.whileTrue(new InstantCommand(m_arm2::armNintey));
        // m_operatorController.lb.whileTrue(new InstantCommand(()->m_arm2.runArm(-0.6)));
        // m_operatorController.lb.whileTrue(new InstantCommand(()->m_arm2.runArm(0.0)));
        // m_operatorController.rb.whileTrue(new InstantCommand(()->m_arm2.runArm(0.2)));
        // m_operatorController.rb.whileFalse(new InstantCommand(()->m_arm2.runArm(0.0)));
        // m_operatorController.back.toggleOnTrue(new CurrentZero2(m_arm2));
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
        // m_drive.resetOdometry(autoChooser.get().getInitialPose());
        return autoChooser.get();
    }
}
