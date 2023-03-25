// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.lib.beaklib.drive.BeakDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalance extends PIDCommand {
    private boolean m_continuous;

    /** Creates a new AutoBalance. */
    public AutoBalance(BeakDrivetrain drivetrain, boolean continuous) {
        super(
            // The controller that the command will use
            new PIDController(1.2, 0, 0), // 1.25
            // This should return the measurement
            () -> drivetrain.getGyroPitchRotation2d().getRadians(),
            // This should return the setpoint (can also be a constant)
            () -> 0,
            // This uses the output
            output -> {
                drivetrain.drive(new ChassisSpeeds(
                    -output,
                    0.,
                    0.));
            });
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here.
        addRequirements(drivetrain);
        getController().setTolerance(Units.degreesToRadians(1.0));

        m_continuous = continuous;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_continuous ? false : getController().atSetpoint();
    }
}
