// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.drive.swerve;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.beaklib.encoder.BeakCANCoder;
import frc.lib.beaklib.motor.BeakTalonFX;

/** SDS MK4i Swerve Module. */
public class BeakMk4iSwerveModule extends BeakSwerveModule {
    /**
     * Construct a new Mk4i Swerve Module.
     * 
     * @param config
     *            {@link SwerveModuleConfiguration} containing
     *            details of the module.
     */
    public BeakMk4iSwerveModule(SwerveModuleConfiguration config) {
        super(config);
        m_driveMotor = new BeakTalonFX(config.driveMotorID, config.CANBus);
        m_turningMotor = new BeakTalonFX(config.turnMotorID, config.CANBus);
        m_turningEncoder = new BeakCANCoder(config.turnEncoderID, config.CANBus);

        super.setup(config);
    }

    public void configDriveMotor(SwerveModuleConfiguration config) {
        super.configDriveMotor(config);

        // Prevent huge CAN spikes
        m_driveMotor.setStatusPeriod(StatusFrameEnhanced.Status_1_General.value, 19);
        m_driveMotor.setStatusPeriod(StatusFrameEnhanced.Status_2_Feedback0.value, 19);
        m_driveMotor.setStatusPeriod(StatusFrameEnhanced.Status_4_AinTempVbat.value, 253);
        m_driveMotor.setStatusPeriod(StatusFrameEnhanced.Status_6_Misc.value, 59);
    }

    public void configTurningMotor(SwerveModuleConfiguration config) {
        super.configTurningMotor(config);

        // Prevent huge CAN spikes
        m_turningMotor.setStatusPeriod(StatusFrameEnhanced.Status_1_General.value, 19);
        m_turningMotor.setStatusPeriod(StatusFrameEnhanced.Status_2_Feedback0.value, 19);
        m_turningMotor.setStatusPeriod(StatusFrameEnhanced.Status_4_AinTempVbat.value, 253);
        m_turningMotor.setStatusPeriod(StatusFrameEnhanced.Status_6_Misc.value, 59);
    }

    @Override
    public double getTurningEncoderRadians() {
        double angle = m_turningEncoder.getAbsolutePosition();
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Calculate Arb Feed Forward for drive motor
        double arbFeedforward = m_feedforward.calculate(desiredState.speedMetersPerSecond);

        m_driveMotor.setVelocityNU(
            desiredState.speedMetersPerSecond / 10.0 / driveEncoderDistancePerPulse,
            arbFeedforward,
            0);

        // Set the turning motor to the correct position.
        setAngle(desiredState.angle.getDegrees());
    }
}