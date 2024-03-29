// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.lib.beaklib.drive.swerve.BeakSwerveDrivetrain;
import frc.lib.beaklib.drive.swerve.BeakSwerveDrivetrain.SnapDirection;
import frc.robot.OneMechanism;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SnapToAngle extends PIDCommand {
    private final BooleanSupplier m_interruptCondition;
    private final BeakSwerveDrivetrain m_drivetrain;

    private final boolean m_updateBeacon;

    /** Creates a new KeepAngle. */
    public SnapToAngle(
        SnapDirection target,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        BooleanSupplier interruptCondition,
        boolean updateBeacon,
        BeakSwerveDrivetrain drivetrain) {
        super(
            // The controller that the command will use
            new PIDController(
                2.0,
                0,
                0),
            // This should return the measurement
            () -> drivetrain.getRotation2d().getRadians(),
            // This should return the setpoint (can also be a constant)
            () -> target.Angle.getRadians(),
            // This uses the output
            output -> {
                // Use the output here
                drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSupplier.getAsDouble(),
                    ySupplier.getAsDouble(),
                    output,
                    drivetrain.getRotation2d()));
            });
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);

        // Configure additional PID options by calling `getController` here.
        getController().enableContinuousInput(-Math.PI, Math.PI);

        if (updateBeacon) {
            drivetrain.setSnapDirection(target);
        }

        m_drivetrain = drivetrain;
        m_interruptCondition = interruptCondition;
        m_updateBeacon = updateBeacon;
    }

    @Override
    public void initialize() {
        super.initialize();

        if (m_updateBeacon) {
            OneMechanism.setSnappedMode(true);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_interruptCondition.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        if (m_updateBeacon) {
            OneMechanism.setSnappedMode(false);
            m_drivetrain.setSnapDirection(SnapDirection.NONE);
        }
    }
}
