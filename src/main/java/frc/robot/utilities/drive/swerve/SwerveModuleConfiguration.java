// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.drive.swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utilities.drive.swerve.SdsModuleConfiguration.ModuleType;
import frc.robot.utilities.units.Distance;

/** Class containing general configuration for a {@link BeakSwerveModule}. */
public class SwerveModuleConfiguration {
    public int driveMotorID;
    public int turnMotorID;
    public int turnEncoderID;
    public double angleOffset;
    public Translation2d moduleLocation;

    public double driveGearRatio;
    public double turnGearRatio;

    public Distance wheelDiameter;

    public boolean driveInverted;
    public boolean turnInverted;

    public double drive_kP;
    public double turn_kP;

    public double allowedError;

    public int turnCurrentLimit;

    public int driveSupplyCurrentLimit;
    public int driveStatorCurrentLimit;

    public String CANBus;

    public SimpleMotorFeedforward feedforward;

    public ModuleType moduleType;

    /**
     * Generate a new Swerve Module configuration.
     * 
     * @param driveMotorID   CAN ID of the drive motor.
     * @param turnMotorID    CAN ID of the turning motor.
     * @param turnEncoderID  CAN ID of the CANCoder.
     * @param angleOffset    Offset of the CANCoder, in radians.
     * @param moduleLocation The module's translation relative to the robot center.
     * @param driveConfig    {@link SwerveDrivetrainConfiguration} for the
     *                       drivetrain this module lies on.
     */
    public SwerveModuleConfiguration(
            int driveMotorID,
            int turnMotorID,
            int turnEncoderID,
            double angleOffset,
            Translation2d moduleLocation,
            SwerveDrivetrainConfiguration driveConfig) {
        this.driveMotorID = driveMotorID;
        this.turnMotorID = turnMotorID;
        this.turnEncoderID = turnEncoderID;
        this.angleOffset = angleOffset;
        this.moduleLocation = moduleLocation;

        this.drive_kP = driveConfig.drive_kP;
        this.turn_kP = driveConfig.turn_kP;

        this.allowedError = driveConfig.allowedClosedLoopError;

        this.turnCurrentLimit = driveConfig.turnCurrentLimit;
        this.driveSupplyCurrentLimit = driveConfig.driveSupplyLimit;
        this.driveStatorCurrentLimit = driveConfig.driveStatorLimit;

        this.CANBus = driveConfig.CANBus;

        this.feedforward = driveConfig.feedforward;

        this.wheelDiameter = driveConfig.moduleConfiguration.wheelDiameter;

        this.driveGearRatio = driveConfig.moduleConfiguration.driveGearRatio;
        this.driveInverted = driveConfig.moduleConfiguration.driveInverted;

        this.turnGearRatio = driveConfig.moduleConfiguration.turnGearRatio;
        this.turnInverted = driveConfig.moduleConfiguration.turnInverted;

        this.moduleType = driveConfig.moduleConfiguration.moduleType;
    }
}
