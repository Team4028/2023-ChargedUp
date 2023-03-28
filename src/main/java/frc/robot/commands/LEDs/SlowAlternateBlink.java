// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.Color;

public class SlowAlternateBlink extends CommandBase {
    private LEDs m_leds;
    private Color firstColor;
    private Color secondColor;
    private double seconds;

    /**
     * 
     * @param firstColor
     *            The first color to alternate between
     * @param secondColor
     *            The second color to alternate between
     * @param secondsWait
     *            The amount of seconds to wait before swiching colors (>= 0)
     * @param leds
     *            the LEDs subsystem to use
     */
    public SlowAlternateBlink(Color firstColor, Color secondColor, double secondsWait, LEDs leds) {
        m_leds = leds;
        this.firstColor = firstColor;
        this.secondColor = secondColor;
        seconds = secondsWait >= 0 ? secondsWait : 0;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(leds);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        new SequentialCommandGroup(
            new InstantCommand(() -> m_leds.setColor(firstColor)),
            new WaitCommand(seconds),
            new InstantCommand(() -> m_leds.setColor(secondColor)),
            new WaitCommand(seconds)).schedule();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_leds.setColor(firstColor);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
