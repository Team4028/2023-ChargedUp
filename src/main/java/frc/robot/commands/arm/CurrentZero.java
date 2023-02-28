// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arms.Arm;

/** An example command that uses an example subsystem. */
public class CurrentZero extends CommandBase {
    private Arm m_arm;
    private double vbus;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem
     *            The subsystem used by this command.
     * @return
     */
    public CurrentZero(Arm arm, double vbus) {
        m_arm = arm;
        this.vbus = vbus;
        
        addRequirements(arm);
        // Use addRequirements() here to declare subsystem dependencies.

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_arm.runArm(vbus);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_arm.zeroEncoder();
        m_arm.runArm(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_arm.getMotorCurrent() >= 25;
    }
}
