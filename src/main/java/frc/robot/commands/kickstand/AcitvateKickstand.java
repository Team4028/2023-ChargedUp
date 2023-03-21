// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.kickstand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OneMechanism;
import frc.robot.subsystems.kickstand.Kickstand;

public class AcitvateKickstand extends CommandBase {
    private Kickstand m_kickstand;

    /** Creates a new RunKickstandToPosition. */
    public AcitvateKickstand(Kickstand kickstand) {
        m_kickstand = kickstand;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(kickstand);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        OneMechanism.setThrowOnGround(true);
        m_kickstand.activateKickstand();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
