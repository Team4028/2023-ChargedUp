// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.OneMechanism.GamePieceMode;
import frc.robot.OneMechanism.ScoringPositions;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;
    // private LEDs m_leds;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        Logger logger = Logger.getInstance();
        // m_leds = LEDs.getInstance();
        // Record metadata
        logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        // Set up data receivers & replay source
        switch (Constants.currentMode) {
            // Running on a real robot, log to a USB stick
            case REAL:
                logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
                logger.addDataReceiver(new NT4Publisher());
                break;

            // Running a physics simulator, log to local folder
            case SIM:
                logger.addDataReceiver(new WPILOGWriter(""));
                logger.addDataReceiver(new NT4Publisher());
                break;

            // Replaying a log, set up replay source
            case REPLAY:
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                logger.setReplaySource(new WPILOGReader(logPath));
                logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        // Start AdvantageKit logger
        logger.start();

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything in
        // the Command-based framework to work.
        CommandScheduler.getInstance().run();

        SmartDashboard.putBoolean("Score Mode", OneMechanism.getScoreMode());
        SmartDashboard.putNumber("Node", OneMechanism.getCurrentNode().GridID);
        SmartDashboard.putBoolean("Mode", OneMechanism.getGamePieceMode() == GamePieceMode.PURPLE_CUBE);

        SmartDashboard.putString("Target Position", OneMechanism.getScoringPosition().name());

        SmartDashboard.putBoolean("WALL", (OneMechanism.getScoringPosition() == ScoringPositions.OLD_WALL || OneMechanism.getScoringPosition() == ScoringPositions.ACQUIRE_DOUBLE_SUBSTATION_CUBE));
        SmartDashboard.putBoolean("SLIDE", (OneMechanism.getScoringPosition() == ScoringPositions.ACQUIRE_SINGLE_SUBSTATION));
        SmartDashboard.putBoolean("FLOOR", (OneMechanism.getScoringPosition() == ScoringPositions.ACQUIRE_FLOOR_CUBE || OneMechanism.getScoringPosition() == ScoringPositions.ACQUIRE_FLOOR_CONE_TIPPED));
        SmartDashboard.putBoolean("UPRIGHT", (OneMechanism.getScoringPosition() == ScoringPositions.ACQUIRE_FLOOR_CONE_UPRIGHT));
        SmartDashboard.putBoolean("STOW", (OneMechanism.getScoringPosition() == ScoringPositions.STOWED));
        SmartDashboard.putBoolean("LOW", (OneMechanism.getScoringPosition() == ScoringPositions.SCORE_LOW_CONE || OneMechanism.getScoringPosition() == ScoringPositions.SCORE_LOW_CUBE));
        SmartDashboard.putBoolean("MID", (OneMechanism.getScoringPosition() == ScoringPositions.SCORE_MID_CONE || OneMechanism.getScoringPosition() == ScoringPositions.SCORE_MID_CUBE));
        SmartDashboard.putBoolean("HIGH", (OneMechanism.getScoringPosition() == ScoringPositions.SCORE_HIGH_CONE || OneMechanism.getScoringPosition() == ScoringPositions.SCORE_HIGH_CUBE));

        SmartDashboard.putNumber("Jerk", OneMechanism.getJerk());
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        OneMechanism.setScoreMode(false);
        OneMechanism.setClimbMode(false);
        OneMechanism.setSnappedMode(false);
        OneMechanism.setIdle();
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        OneMechanism.setActive();
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        OneMechanism.setActive();
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }
}
