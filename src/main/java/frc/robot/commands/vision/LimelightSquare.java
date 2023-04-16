// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.beaklib.drive.BeakDrivetrain;
import frc.robot.OneMechanism;
import frc.robot.utilities.LimelightHelpers;

public class LimelightSquare extends CommandBase {
    /**
     * The threshold for the TY value below which the cube will be considered
     * "picked up"
     */
    private static final double CUBE_PICKUP_THRESHOLD = -12;

    /**
     * The threshold for the TY value below which the cone will be considered
     * "picked up"
     */
    private static final double CONE_PICKUP_THRESHOLD = -1;

    private final boolean m_cone, m_continuous;
    private boolean m_hasReachedThreshold = false;

    private final DoubleSupplier m_xSupplier, m_ySupplier;

    private final BeakDrivetrain m_drivetrain;

    private PIDController m_controller;

    /** Creates a new NewLimelightSquare. */
    public LimelightSquare(boolean cone, boolean continuous, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
        BeakDrivetrain drivetrain) {
        m_cone = cone;
        m_continuous = continuous;

        m_xSupplier = xSupplier;
        m_ySupplier = ySupplier;

        m_drivetrain = drivetrain;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_controller = new PIDController(DriverStation.isAutonomousEnabled() ? 7.5 : 5.0, 0., 0.);
        m_controller.reset();

        OneMechanism.setLocked(true);

        LimelightHelpers.setPipelineIndex("", m_cone ? 1 : 0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!m_hasReachedThreshold)
            m_hasReachedThreshold = (LimelightHelpers
                .getTY("") < (m_cone ? CONE_PICKUP_THRESHOLD : CUBE_PICKUP_THRESHOLD));

        double rotationOutput = m_controller.calculate(Units.degreesToRadians(LimelightHelpers.getTX("")));

        m_drivetrain.drive(
            new ChassisSpeeds(
                m_xSupplier.getAsDouble(),
                m_ySupplier.getAsDouble(),
                m_continuous || !m_hasReachedThreshold ? rotationOutput : 0.));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        OneMechanism.setLocked(false);

        m_drivetrain.drive(new ChassisSpeeds());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
