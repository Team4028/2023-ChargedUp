// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.drive.swerve;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.utilities.encoder.BeakCANCoder;
import frc.robot.utilities.motor.BeakTalonFX;

/** SDS MK4i Swerve Module that has not been characterized. */
public class BeakUncharacterizedMk4iSwerveModule extends BeakSwerveModule {
    /**
     * Construct a new Mk4i Swerve Module.
     * 
     * @param config {@link SwerveModuleConfiguration} containing
     *               details of the module.
     */
    public BeakUncharacterizedMk4iSwerveModule(SwerveModuleConfiguration config) {
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
        // TODO: put in config
        m_driveMotor.set(desiredState.speedMetersPerSecond / Units.feetToMeters(16.3));

        // Set the turning motor to the correct position.
        setAngle(desiredState.angle.getDegrees());
    }
}
