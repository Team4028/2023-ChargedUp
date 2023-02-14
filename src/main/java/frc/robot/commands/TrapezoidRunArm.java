// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;
import frc.robot.subsystems.arms.Arm;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrapezoidRunArm extends TrapezoidProfileCommand {
  /** Creates a new TrapezoidRunArm. */
  public TrapezoidRunArm(double maxVel, double maxAccel, double startRotations, double endRotations, Arm m_arm) {
    super(
        // The motion profile to be executed
        new TrapezoidProfile(
            // The motion profile constraints
            new TrapezoidProfile.Constraints(maxVel, maxAccel),
            // Goal state
            new TrapezoidProfile.State(endRotations,0),
            // Initial state
            new TrapezoidProfile.State(startRotations,0)),
        state -> {
            m_arm.runToPosition(state.position);
          // Use current trajectory state here
        });
        m_arm.setDistanceToTravel(endRotations - startRotations);
  }
}
