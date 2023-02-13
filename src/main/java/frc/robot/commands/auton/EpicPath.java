// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import frc.robot.utilities.drive.BeakDrivetrain;
import frc.robot.utilities.drive.Trajectories;
import frc.robot.utilities.units.Distance;

public class EpicPath extends BeakAutonCommand {
    /** Creates a new TestPath. */
    public EpicPath(BeakDrivetrain drivetrain) {
        super.addCommands(
                drivetrain.getTrajectoryCommand(Trajectories.Ball1(drivetrain)),
                drivetrain.getTrajectoryCommand(Trajectories.Ball2(drivetrain)),
                new RotateDrivetrainToTargetPosition(Distance.fromInches(324.), Distance.fromInches(162.), drivetrain));
        super.setInitialPose(Trajectories.Ball1(drivetrain));
    }
}
