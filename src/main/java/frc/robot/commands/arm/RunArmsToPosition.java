// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.SuppliedWaitCommand;
import frc.robot.subsystems.arms.Arm;
import frc.robot.subsystems.arms.LowerArm;
import frc.robot.subsystems.arms.UpperArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunArmsToPosition extends SequentialCommandGroup {
    /** Creates a new RunArmsToPosition. */
    public RunArmsToPosition(Arm.ArmPositions targetPositions, LowerArm lowerArm, UpperArm upperArm) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());

        addCommands(
            new ConditionalCommand(
                // EXTENDING COMMAND
                // Begins extending lower arm,
                // Then waits a period based on the distance needed to travel
                // and then begins extending the upper arm.
                new RunArm(targetPositions.lowerPosition, lowerArm)
                    .alongWith(new SuppliedWaitCommand(
                        () -> lowerArm.getDistanceToTravel() / Constants.ArmConstants.EXTEND_COEFFICIENT)
                            .andThen(new RunArm(targetPositions.upperPosition, upperArm))),
                // RETRACTING COMMAND
                // Begins retracting upper arm,
                // Then waits a period based on the distance needed to travel
                // and then begins retracting the lower arm.
                new RunArm(targetPositions.upperPosition, upperArm)
                    .alongWith(new SuppliedWaitCommand(
                        () -> upperArm.getDistanceToTravel() / Constants.ArmConstants.RETRACT_COEFFICIENT)
                            .andThen(new RunArm(targetPositions.lowerPosition, lowerArm))),
                () -> targetPositions.upperPosition > upperArm.getEncoderPosition()));
    }
}
