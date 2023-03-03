// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.beaklib.drive.BeakDrivetrain;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AddVisionMeasurement extends InstantCommand {
    private final BeakDrivetrain m_drivetrain;
    private final Vision m_vision;
    private final Field2d m_field;

    public AddVisionMeasurement(BeakDrivetrain drivetrain, Vision vision) {
        m_drivetrain = drivetrain;
        m_vision = vision;
        m_field = new Field2d();

        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(drivetrain, vision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_vision.checkAlliance();
        m_drivetrain.addVisionMeasurement(
            m_vision.getLatestEstimatedRobotPose(m_drivetrain.getRotation2d()),
            m_vision.getLatestLatency());
        m_field.setRobotPose(m_drivetrain.getPoseMeters());
        SmartDashboard.putData("AprilTag thing", m_field);
    }
}
