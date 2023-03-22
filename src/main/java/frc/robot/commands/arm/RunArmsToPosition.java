// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.manipulator.Wrist;
import frc.robot.OneMechanism;
import frc.robot.OneMechanism.ScoringPositions;
import frc.robot.subsystems.arms.LowerArm;
import frc.robot.subsystems.arms.UpperArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunArmsToPosition extends SequentialCommandGroup {

    /** Creates a new RunArmsToPosition. */
    public RunArmsToPosition(ScoringPositions targetPos,
        LowerArm lowerArm, UpperArm upperArm, Wrist wrist) {
        
        //set the scoring pos for OneMechanism to track
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            new ConditionalCommand(
                // EXTENDING COMMAND
                // Begins extending lower arm,
                // Then waits a period based on the distance needed to travel
                // and then begins extending the upper arm.
                new RunArmProfiled(lowerArm.MaxVel, lowerArm.MaxAccel, lowerArm.getEncoderPosition(), targetPos.lowerPosition,
                    lowerArm)
                        /*
                         * .alongWith(new SuppliedWaitCommand(() -> lowerArm.getDistanceToTravel() /
                         * Constants.ArmConstants.EXTEND_COEFFICIENT)
                         */
                        .andThen(new RunArmProfiled(upperArm.MaxVel, upperArm.MaxAccel, upperArm.getEncoderPosition(),
                            targetPos.upperPosition, upperArm))
                        .alongWith(wrist.runToAngle(targetPos.wristAngle)),
                // RETRACTING COMMAND
                // Begins retracting upper arm,
                // Then waits a period based on the distance needed to travel
                // and then begins retracting the lower arm.
                new RunArmProfiled(upperArm.MaxVel, upperArm.MaxAccel, upperArm.getEncoderPosition(), targetPos.upperPosition,
                    upperArm)
                        .raceWith(wrist.runToAngle(targetPos.wristAngle))
                        /*
                         * .alongWith(new SuppliedWaitCommand(() -> upperArm.getDistanceToTravel() /
                         * Constants.ArmConstants.RETRACT_COEFFICIENT)
                         */
                        .andThen(new RunArmProfiled(lowerArm.MaxVel, lowerArm.MaxAccel, lowerArm.getEncoderPosition(),
                            targetPos.lowerPosition, lowerArm)),
                () -> targetPos.lowerPosition > lowerArm.getEncoderPosition()),
                new InstantCommand(() -> OneMechanism.setScoringPosition(targetPos)));
        addRequirements(upperArm, lowerArm, wrist);
    }
}
