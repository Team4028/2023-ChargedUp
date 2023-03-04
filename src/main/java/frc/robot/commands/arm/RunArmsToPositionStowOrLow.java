// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.OneMechanism;
import frc.robot.OneMechanism.ScoringPositions;
import frc.robot.subsystems.arms.LowerArm;
import frc.robot.subsystems.arms.UpperArm;
import frc.robot.subsystems.manipulator.Wrist;

public class RunArmsToPositionStowOrLow extends SequentialCommandGroup {

    /** Creates a new RunArmsToPositionStowOrLow. */
    public RunArmsToPositionStowOrLow(ScoringPositions targetPos, LowerArm lowerArm, UpperArm upperArm, Wrist wrist) {
        ScoringPositions currentScoringPosition = OneMechanism.getScoringPosition();
        addCommands(
            new ConditionalCommand(
                new RunArmsToPosition(ScoringPositions.INTERMEDIATE_LOW, lowerArm, upperArm, wrist)
                    .andThen(new WaitCommand(.125))
                    .andThen(new RunArmsToPosition(targetPos, lowerArm, upperArm, wrist)),
                new RunArmsToPosition(targetPos, lowerArm, upperArm, wrist),
                () -> (currentScoringPosition.equals(ScoringPositions.ACQUIRE_FLOOR_TIPPED_CONE)
                    || currentScoringPosition.equals(ScoringPositions.ACQUIRE_FLOOR_CUBE)
                    || currentScoringPosition.equals(ScoringPositions.STOWED))
                    && (targetPos.equals(ScoringPositions.STOWED)
                        || targetPos.equals(ScoringPositions.ACQUIRE_FLOOR_TIPPED_CONE)
                        || targetPos.equals(ScoringPositions.ACQUIRE_FLOOR_CUBE))));
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(lowerArm, upperArm, wrist);

    }
}
