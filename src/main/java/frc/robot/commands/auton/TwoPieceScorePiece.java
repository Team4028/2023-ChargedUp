// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import frc.robot.utilities.drive.BeakDrivetrain;
import frc.robot.utilities.drive.Trajectories;

public class TwoPieceScorePiece extends BeakAutonCommand {
    /** Creates a new TestPath. */
    public TwoPieceScorePiece(BeakDrivetrain drivetrain) {
        super.addCommands(
                drivetrain.getTrajectoryCommand(Trajectories.TwoPieceScorePiece(drivetrain)));
        super.setInitialPose(Trajectories.TwoPieceScorePiece(drivetrain));
    }
}
