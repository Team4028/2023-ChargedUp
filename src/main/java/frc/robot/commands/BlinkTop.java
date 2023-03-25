// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.Color;

public class BlinkTop extends CommandBase {
    private SequentialCommandGroup cmd;
    private LEDs m_leds;
    /** Creates a new BlinkTop. */
    public BlinkTop(Color color, double speed, LEDs leds) {
        cmd = leds.blinkTop(color, speed);
        m_leds = leds;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(leds);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        cmd.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        cmd.cancel();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !m_leds.getSnappedState();
    }
}
