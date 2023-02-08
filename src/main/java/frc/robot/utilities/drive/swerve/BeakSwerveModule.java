// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.drive.swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.encoder.BeakAbsoluteEncoder;
import frc.robot.utilities.motor.BeakMotorController;

/** Base class for any non-differential swerve module. */
public class BeakSwerveModule {
    protected double turnCPR;
    protected int bruh;

    // Calculated from Drive CPR.
    protected double driveEncoderDistancePerPulse;

    protected SimpleMotorFeedforward m_feedforward;

    protected BeakMotorController m_driveMotor;
    protected BeakMotorController m_turningMotor;
    protected BeakAbsoluteEncoder m_turningEncoder;

    /**
     * Construct a new Swerve Module.
     * 
     * @param config {@link SwerveModuleConfiguration} containing
     *               details of the module.
     */
    public BeakSwerveModule(SwerveModuleConfiguration config) {
    }

    /**
     * Call this function in a subclass AFTER setting up motors and encoders
     */
    public void setup(SwerveModuleConfiguration config) {
        turnCPR = config.turnGearRatio * m_turningMotor.getVelocityEncoderCPR();
        driveEncoderDistancePerPulse = (config.wheelDiameter.getAsMeters() * Math.PI)
                * config.driveGearRatio / m_driveMotor.getVelocityEncoderCPR();

        m_feedforward = config.feedforward;

        bruh = config.driveMotorID;

        configTurningEncoder(config);
        configTurningMotor(config);
        configDriveMotor(config);
    }

    public void configDriveMotor(SwerveModuleConfiguration config) {
        m_driveMotor.restoreFactoryDefault();

        m_driveMotor.setBrake(true);
        m_driveMotor.setInverted(config.driveInverted);

        // Prevent the motors from drawing several hundred amps of current,
        // and allow them to run at the same speed even when voltage drops.
        m_driveMotor.setVoltageCompensationSaturation(12.0);
        m_driveMotor.setSupplyCurrentLimit(config.driveSupplyCurrentLimit);
        m_driveMotor.setStatorCurrentLimit(config.driveStatorCurrentLimit);

        // Configure PID
        m_driveMotor.setP(config.drive_kP, 0);

        m_driveMotor.setDistancePerPulse(config.wheelDiameter, config.driveGearRatio);
    }

    public void configTurningMotor(SwerveModuleConfiguration config) {
        m_turningMotor.restoreFactoryDefault();

        m_turningMotor.setBrake(true);
        m_turningMotor.setInverted(config.turnInverted);

        // Initialize the encoder's position--MUST BE DONE AFTER
        // CONFIGURING TURNING ENCODER!
        resetTurningMotor();

        // Generally, turning motor current draw isn't a problem.
        // This is done to prevent stalls from killing the motor.
        m_turningMotor.setSupplyCurrentLimit(config.turnCurrentLimit);

        m_turningMotor.setAllowedClosedLoopError(config.allowedError, 0);

        m_turningMotor.setP(config.turn_kP, 0);
        m_turningMotor.setD(0.3, 0); // TODO: Add to config
    }

    public void configTurningEncoder(SwerveModuleConfiguration config) {
        m_turningEncoder.setAbsoluteOffset(Units.radiansToDegrees(config.angleOffset));

        // Prevent huge CAN spikes
        m_turningEncoder.setDataFramePeriod(101);
    }

    /* State Management */

    /**
     * Get the module's current state.
     * 
     * @return Current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                m_driveMotor.getVelocityNU() * driveEncoderDistancePerPulse * 10., // TODO
                // m_driveMotor.getRate() / 10.,
                new Rotation2d(getTurningEncoderRadians())); // FUTURE: Using Absolute reverses some wheels.
    }

    /**
     * Get the module's current position.
     * 
     * @return Current position of the module.
     */
    public SwerveModulePosition getPosition() {
        double positionDistancePerPulse = driveEncoderDistancePerPulse * m_driveMotor.getVelocityEncoderCPR() / m_driveMotor.getPositionEncoderCPR();
        return new SwerveModulePosition(
            m_driveMotor.getPositionNU() * positionDistancePerPulse,
            new Rotation2d(getTurningEncoderRadians())
        );
    }

    /**
     * Set the desired state for the module, and run the motors.
     * 
     * @param desiredState Desired {@link SwerveModuleState} containing the speed
     *                     and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the state to avoid spinning more than 90 degrees.
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState,
                new Rotation2d(getTurningEncoderRadians()));

        // Calculate Arb Feed Forward for drive motor
        // TODO: calc from SysId
        // NOTE: feedforward MUST be in meters!
        double arbFeedforward = m_feedforward.calculate(optimizedState.speedMetersPerSecond);

        m_driveMotor.setVelocityNU(
        // m_driveMotor.setRate(
                optimizedState.speedMetersPerSecond / 10.0 / driveEncoderDistancePerPulse,
                // optimizedState.speedMetersPerSecond,
                arbFeedforward,
                0);

        SmartDashboard.putNumber("bruh " + bruh, m_driveMotor.getOutputVoltage());

        // Set the turning motor to the correct position.
        setAngle(optimizedState.angle.getDegrees());
    }

    /** Encoders & Heading */

    /**
     * Set the turning motor's position to match the reported
     * angle from the CANCoder.
     */
    public void resetTurningMotor() {
        m_turningMotor.setEncoderPositionNU(
                Math.toDegrees(getTurningEncoderRadians()) / 360.0 * turnCPR);
    }

    /**
     * Get the angle of the wheel.
     * 
     * @return Angle of the wheel in radians.
     */
    public double getTurningEncoderRadians() {
        double angle = m_turningEncoder.getPosition();
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    /**
     * Zero all encoders, in case things have gone bad
     */
    public void resetEncoders() {
        m_driveMotor.setEncoderPositionNU(0);
        m_turningMotor.setEncoderPositionNU(0);
    }

    /**
     * Set the wheel's angle.
     * 
     * @param newAngle Angle to turn the wheel to, in degrees.
     */
    public void setAngle(double newAngle) {
        // Does some funky stuff to do the cool thing
        double currentSensorPosition = m_turningMotor.getPositionNU() * 360.0
                / turnCPR;
        double remainder = Math.IEEEremainder(currentSensorPosition, 360.0);
        double newAngleDemand = newAngle + currentSensorPosition - remainder;

        if (newAngleDemand - currentSensorPosition > 180.1) {
            newAngleDemand -= 360.0;
        } else if (newAngleDemand - currentSensorPosition < -180.1) {
            newAngleDemand += 360.0;
        }

        // SmartDashboard.putNumber("state " + bruh, newAngleDemand / 360. * turnCPR);

        m_turningMotor.setPositionNU(newAngleDemand / 360.0 * turnCPR);
    }

    public static BeakSwerveModule fromSwerveModuleConfig(SwerveModuleConfiguration config) {
        switch (config.moduleType) {
            case MK4i:
                return new BeakMk4iSwerveModule(config);
            case MK2:
                return new BeakMk2SwerveModule(config);
            default:
                return null;
        }
    }
}
