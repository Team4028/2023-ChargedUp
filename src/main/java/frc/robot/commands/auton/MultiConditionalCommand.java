// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class MultiConditionalCommand<E extends Enum<E>> extends Command {
    private Map<E, Command> m_commandMap;
    private Supplier<E> m_valueSupplier;

    private Command m_currentCommand;

    /** Creates a new MultiConditionalCommand.
     * @param commandMap <Enum Value, CommandToRun>
     * @param supplier Should return the enum value at runtime.
    */
    public MultiConditionalCommand(Map<E, Command> commandMap, Supplier<E> supplier, Subsystem... requirements) {
        m_commandMap = commandMap;
        m_valueSupplier = supplier;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(requirements);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_currentCommand = m_commandMap.get(m_valueSupplier.get());
        m_currentCommand.initialize();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_currentCommand.execute();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_currentCommand.end(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_currentCommand.isFinished();
    }
}
