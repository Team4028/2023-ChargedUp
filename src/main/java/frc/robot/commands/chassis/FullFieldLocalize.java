// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.beaklib.drive.BeakDrivetrain;
import frc.robot.subsystems.Vision;

public class FullFieldLocalize extends SequentialCommandGroup {
    /** Localize the drivetrain's pose using cameras both on the front and back. */
    public FullFieldLocalize(BeakDrivetrain drivetrain, Vision frontVision, Vision rearVision) {
        super.addCommands(
            new AddVisionMeasurement(drivetrain, frontVision),
            new AddVisionMeasurement(drivetrain, rearVision)
        );
    }
}
