// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.OneMechanism;
import frc.robot.OneMechanism.ScoringPositions;
import frc.robot.subsystems.arms.LowerArm;
import frc.robot.subsystems.arms.UpperArm;
import frc.robot.subsystems.manipulator.Wrist;

public class RunArmsSafely extends SequentialCommandGroup {

    /** Creates a new RunArmsSafely. */
    public RunArmsSafely(ScoringPositions targetPos, LowerArm lowerArm, UpperArm upperArm, Wrist wrist) {
        addCommands(
            new ConditionalCommand(
                new RunArmsWithPID(ScoringPositions.INTERMEDIATE_LOW, lowerArm, upperArm, wrist)
                    .andThen(new RunArmsWithPID(targetPos, lowerArm, upperArm, wrist)),
                new RunArmsWithPID(targetPos, lowerArm, upperArm, wrist),
                () -> ((OneMechanism.getScoringPosition().equals(ScoringPositions.ACQUIRE_FLOOR_CONE_TIPPED)
                    || OneMechanism.getScoringPosition().equals(ScoringPositions.ACQUIRE_FLOOR_CUBE)
                    || OneMechanism.getScoringPosition().equals(ScoringPositions.STOWED))
                    && (targetPos.equals(ScoringPositions.STOWED)
                        || targetPos.equals(ScoringPositions.ACQUIRE_FLOOR_CONE_TIPPED)
                        || targetPos.equals(ScoringPositions.ACQUIRE_FLOOR_CUBE))
                    && targetPos != OneMechanism.getScoringPosition())));
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(lowerArm, upperArm, wrist);

    }
}
