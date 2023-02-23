// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.arms.Arm;
import frc.robot.subsystems.arms.LowerArm;
import frc.robot.subsystems.arms.UpperArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunArmsToPosition extends SequentialCommandGroup {
    private double maxVel = 7000;
    private double maxAccel = 14000;
    private UpperArm m_upperarm = UpperArm.getInstance();
    private LowerArm m_lowerarm = LowerArm.getInstance();
    /** Creates a new RunArmsToPosition. */
    public RunArmsToPosition(Arm.ArmPositions targetArmPositions, Wrist.WristPositions targetWristPosition, LowerArm lowerArm, UpperArm upperArm, Wrist wrist) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        
        addCommands(
                new ConditionalCommand(
                        // EXTENDING COMMAND
                        // Begins extending lower arm,
                        // Then waits a period based on the distance needed to travel
                        // and then begins extending the upper arm.
                        new TrapezoidRunArm(maxVel, maxAccel, m_lowerarm.getEncoderPosition(), targetArmPositions.lowerPosition, lowerArm) 
                                /*.alongWith(new SuppliedWaitCommand(() -> lowerArm.getDistanceToTravel() / Constants.ArmConstants.EXTEND_COEFFICIENT)*/
                                    .andThen(new TrapezoidRunArm(maxVel, maxAccel, m_upperarm.getEncoderPosition(), targetArmPositions.upperPosition, upperArm).alongWith(new RunWrist(targetWristPosition.position, wrist))),
                        // RETRACTING COMMAND
                        // Begins retracting upper arm,
                        // Then waits a period based on the distance needed to travel
                        // and then begins retracting the lower arm.
                        new TrapezoidRunArm(maxVel, maxAccel, m_upperarm.getEncoderPosition(), targetArmPositions.upperPosition, upperArm).alongWith(new RunWrist(targetWristPosition.position, wrist))
                                /*.alongWith(new SuppliedWaitCommand(() -> upperArm.getDistanceToTravel() / Constants.ArmConstants.RETRACT_COEFFICIENT)*/
                                    .andThen(new TrapezoidRunArm(maxVel, maxAccel, m_lowerarm.getEncoderPosition(), targetArmPositions.lowerPosition, lowerArm).alongWith(new RunWrist(targetWristPosition.position, wrist))),
                        () -> targetArmPositions.upperPosition > upperArm.getEncoderPosition()));
    }
}
