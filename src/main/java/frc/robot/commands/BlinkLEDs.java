// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LEDs;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlinkLEDs extends SequentialCommandGroup {
    /** Creates a new BlinkLEDs. */
    public BlinkLEDs(LEDs leds) {
        addCommands(
            // TODO: I hate this why is there no way to repeat a command a few times
            leds.setOff(),
            new WaitCommand(0.2),
            leds.setLEDs(),
            new WaitCommand(0.2),
            leds.setOff(),
            new WaitCommand(0.2),
            leds.setLEDs(),
            new WaitCommand(0.2),
            leds.setOff(),
            new WaitCommand(0.2),
            leds.setLEDs(),
            new WaitCommand(0.2),
            leds.setOff(),
            new WaitCommand(0.2),
            leds.setLEDs()
        );
    }
}