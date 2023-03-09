// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.arm;

import frc.robot.subsystems.arms.Arm;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunArm extends TrapezoidProfileCommand {
    private Arm m_arm;
    private double m_endPos;
    private double m_startPos;
    /** Creates a new RunArm. */
    public RunArm(double maxVel, double maxAccel, double startPos, double endPos, Arm arm) {
        super(
            // The motion profile to be executed
            new TrapezoidProfile(
                // The motion profile constraints
                new TrapezoidProfile.Constraints(maxVel, maxAccel),
                // Goal state
                new TrapezoidProfile.State(endPos, 0),
                // Initial state
                new TrapezoidProfile.State(startPos, 0)),
            state -> {
                arm.setDistanceToTravel(Math.abs(endPos - startPos));

                arm.runToPosition(state.position);//, arm.ffmodel.calculate(state.velocity));
                // Use current trajectory state here
                SmartDashboard.putNumber("RunArm Position:", state.position);
                SmartDashboard.putNumber("RunArm Vel", state.velocity);
            });
            
        m_arm = arm;
        m_endPos = endPos;
        m_startPos = startPos;

        // PLEASE READ THE FOLLOWING
        // MUST ALWAYS ADD REQUIREMENTS
        // ALWAYS
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        super.initialize();

        m_arm.setDistanceToTravel(Math.abs(m_endPos - m_startPos));
    }
}
