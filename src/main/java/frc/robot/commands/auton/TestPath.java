// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utilities.drive.BeakDrivetrain;
import frc.robot.utilities.drive.Trajectories;

public class TestPath extends BeakAutonCommand {
    /** Creates a new TestPath. */
    public TestPath(BeakDrivetrain drivetrain) {
        super.addCommands(
                drivetrain.getTrajectoryCommand(Trajectories.TestPath(drivetrain)),
                new RotateDrivetrainToAngle(() -> Rotation2d.fromDegrees(0.), drivetrain, false));
        super.setInitialPose(Trajectories.TestPath(drivetrain));
    }
}
