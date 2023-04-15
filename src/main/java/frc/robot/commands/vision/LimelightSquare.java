// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.lib.beaklib.drive.BeakDrivetrain;
import frc.robot.OneMechanism;
import frc.robot.utilities.LimelightHelpers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LimelightSquare extends PIDCommand {
    /** The threshold for the TY value below which the cube will be considered "picked up" */
    private static final double PICKUP_THRESHOLD = -12;

    private final boolean m_continuous;
    private BooleanSupplier m_cone;

    /** Creates a new LimelightSquare. */
    public LimelightSquare(BooleanSupplier cone, boolean continuous, DoubleSupplier xSupplier, DoubleSupplier ySupplier, BeakDrivetrain drivetrain) {
        super(
            // The ProfiledPIDController used by the command
            // use pidcontroller
            // new ProfiledPIDController(5., 0.0, 0,
            //     new TrapezoidProfile.Constraints(
            //         drivetrain.getPhysics().maxAngularVelocity.getAsRadiansPerSecond() / 2., 1.0)),
            new PIDController(5.0, 0, 0), // 5.0 good
            // This should return the measurement
            () -> Units.degreesToRadians(LimelightHelpers.getTX("limelight")),
            // This should return the goal (can also be a constant)
            () -> 0.,
            // This uses the output
            (output) -> {
                // Use the output (and setpoint, if desired) here
                drivetrain.drive(new ChassisSpeeds(
                    xSupplier.getAsDouble() * drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond(),
                    ySupplier.getAsDouble() * drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond(),
                    output));
            });
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here.
        getController().enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drivetrain);

        m_continuous = continuous;
        m_cone = cone;
    }

    @Override
    public void initialize() {
        super.initialize();
        OneMechanism.setLocked(true);
    }

    @Override
    public void execute() {
        super.execute();
        LimelightHelpers.setPipelineIndex("limelight", m_cone.getAsBoolean() ? 1 : 0);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        OneMechanism.setLocked(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return Math.abs(LimelightHelpers.getTX("")) < (0.5);
        // return false;
        return m_continuous ? false : LimelightHelpers.getTY("") < PICKUP_THRESHOLD;
    }
}
