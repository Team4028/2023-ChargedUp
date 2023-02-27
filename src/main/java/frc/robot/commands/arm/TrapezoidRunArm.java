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
public class TrapezoidRunArm extends TrapezoidProfileCommand {
    /** Creates a new TrapezoidRunArm. */
    public TrapezoidRunArm(double maxVel, double maxAccel, double startRotations, double endInches, Arm m_arm) {
        super(
            // The motion profile to be executed
            new TrapezoidProfile(
                // The motion profile constraints
                new TrapezoidProfile.Constraints(maxVel, maxAccel),
                // Goal state
                new TrapezoidProfile.State(m_arm.inchesToNativeUnits(endInches), 0),
                // Initial state
                new TrapezoidProfile.State(startRotations, 0)),
            state -> {

                m_arm.runToPosition(state.position, m_arm.ffmodel.calculate(state.velocity));
                // Use current trajectory state here
                SmartDashboard.putNumber("Position Trapezoid:", state.position);
                SmartDashboard.putNumber("Trapezoid Vel", state.velocity);
            });
        m_arm.setDistanceToTravel(Math.abs(endInches - startRotations));
    }
}
