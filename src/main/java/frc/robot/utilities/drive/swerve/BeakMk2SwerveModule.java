// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.drive.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.encoder.BeakAnalogInput;
import frc.robot.utilities.motor.BeakSparkMAX;

/** SDS Mk2 Swerve Module. */
public class BeakMk2SwerveModule extends BeakSwerveModule {
    PIDController m_turningPIDController;

    /**
     * Construct a new Mk2 Swerve Module.
     * 
     * @param config {@link SwerveModuleConfiguration} containing
     *               details of the module.
     */
    public BeakMk2SwerveModule(SwerveModuleConfiguration config) {
        super(config);
        m_driveMotor = new BeakSparkMAX(config.driveMotorID);
        m_turningMotor = new BeakSparkMAX(config.turnMotorID);
        m_turningEncoder = new BeakAnalogInput(config.turnEncoderID);

        m_turningPIDController = new PIDController(config.turn_kP, 0., 0.001);

        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        super.setup(config);
    }

    public void configDriveMotor(SwerveModuleConfiguration config) {
        super.configDriveMotor(config);

        m_driveMotor.setD(config.drive_kP * 10., 0);

        // Prevent huge CAN spikes
        // TODO: CAN status frames
    }

    public void configTurningMotor(SwerveModuleConfiguration config) {
        super.configTurningMotor(config);

        // Prevent huge CAN spikes
        // TODO: CAN status frames
    }

    /**
     * Minimize the change in heading the desired swerve module state would require
     * by potentially
     * reversing the direction the wheel spins. If this is used with the
     * PIDController class's
     * continuous input functionality, the furthest a wheel will ever rotate is 90
     * degrees.
     *
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
     * @return Optimized swerve module state.
     */
    public static SwerveModuleState optimize(
            SwerveModuleState desiredState, Rotation2d currentAngle) {
        var delta = desiredState.angle.minus(currentAngle);
        if (Math.abs(delta.getDegrees()) > 90.0) {
            return new SwerveModuleState(
                    -desiredState.speedMetersPerSecond,
                    desiredState.angle.getDegrees() == 0. ? desiredState.angle : desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
        } else {
            return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        }
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the state to avoid spinning more than 90 degrees.
        // TODO: All attempts at an optimize function seem to break stuff for MK2.
        SwerveModuleState optimizedState = desiredState;//optimize(desiredState, new Rotation2d(getTurningEncoderRadians()));
        // SwerveModuleState.optimize(desiredState, new Rotation2d(getTurningEncoderRadians()));

        SmartDashboard.putNumber("bruh " + bruh, desiredState.speedMetersPerSecond);
        SmartDashboard.putNumber("state " + bruh, getState().speedMetersPerSecond);

        m_driveMotor.set(desiredState.speedMetersPerSecond / Units.feetToMeters(14.3));

        // Set the turning motor to the correct position.
        setAngle(optimizedState.angle.getDegrees());
    }

    @Override
    public void setAngle(double angle) { // TODO: Angle motor PID
        double turnOutput = m_turningPIDController.calculate(getTurningEncoderRadians(), Math.toRadians(angle));

        // Calculate the turning motor output from the turning PID controller.
        m_turningMotor.set(turnOutput);

        // SmartDashboard.putNumber("state " + bruh, m_turningMotor.getPositionNU() *
        // 360. / turnCPR);
    }
}
