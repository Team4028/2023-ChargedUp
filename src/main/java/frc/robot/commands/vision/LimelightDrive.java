// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.beaklib.drive.BeakDrivetrain;
import frc.robot.OneMechanism;
import frc.robot.OneMechanism.ScoringPositions;
import frc.robot.commands.chassis.KeepAngle;
import frc.robot.subsystems.manipulator.Gripper;
import frc.robot.utilities.LimelightHelpers;

public class LimelightDrive extends Command {
    private final BeakDrivetrain m_drivetrain;
    private final Gripper m_gripper;

    private KeepAngle m_keepAngleCommand;

    private PIDController m_pidController;
    
    /** Creates a new LimelightDrive. */
    public LimelightDrive(Gripper gripper, BeakDrivetrain drivetrain) {
        m_gripper = gripper;
        m_drivetrain = drivetrain;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(gripper);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_pidController = new PIDController(
            0.1,
            0, 
            0);

        m_pidController.reset();

        m_gripper.runMotorIn();

        double setpoint = OneMechanism.getScoringPosition() == ScoringPositions.FLOOR_CUBE_SEEK ? -8.8 : -16.0;

        m_keepAngleCommand = new KeepAngle(
            true,
            () -> m_pidController.calculate(Units.degreesToRadians(LimelightHelpers.getTY("")), setpoint) * 0.75,
            () -> 0.,
            m_drivetrain);
        
        m_keepAngleCommand.initialize();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_keepAngleCommand.execute();
        m_gripper.runMotorInWithoutReset();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_keepAngleCommand.end(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !LimelightHelpers.getTV("") || m_gripper.hasGamePiece();
    }
}
