// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.beaklib.drive.BeakDrivetrain;

public class KeepAngle extends Command {
    private final DoubleSupplier m_xSupplier;
    private final DoubleSupplier m_ySupplier;
    private final BeakDrivetrain m_drivetrain;

    private final boolean m_fieldRelative;

    private PIDController m_pidController;

    private Rotation2d m_target;
    private final Rotation2d m_offset;

    /** Creates a new KeepAngle. */
    // TODO: javadoc
    public KeepAngle(
        boolean fieldRelative,
        Rotation2d angleOffset,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        BeakDrivetrain drivetrain) {

        m_xSupplier = xSupplier;
        m_ySupplier = ySupplier;
        m_drivetrain = drivetrain;

        m_fieldRelative = fieldRelative;
        m_offset = angleOffset;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
    }

    public KeepAngle(
        boolean fieldRelative,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        BeakDrivetrain drivetrain) {
        this(fieldRelative, new Rotation2d(), xSupplier, ySupplier, drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_pidController = new PIDController(2.0, 0, 0);
        m_pidController.enableContinuousInput(-Math.PI, Math.PI);

        m_pidController.reset();

        m_target = m_drivetrain.getRotation2d().plus(m_offset);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double x = m_xSupplier.getAsDouble();
        double y = m_ySupplier.getAsDouble();
        double rot = m_pidController.calculate(m_drivetrain.getRotation2d().getRadians(), m_target.getRadians());

        m_drivetrain
            .drive(m_fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, m_drivetrain.getRotation2d())
                : new ChassisSpeeds(x, y, rot));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
