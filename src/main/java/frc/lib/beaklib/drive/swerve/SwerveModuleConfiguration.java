// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.drive.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.beaklib.units.Distance;

/** Class containing general configuration for a {@link BeakSwerveModule}. */
public class SwerveModuleConfiguration {
    public final Rotation2d AngleOffset;
    public final Translation2d ModuleLocation;

    public final double DriveGearRatio;
    public final double TurnGearRatio;

    public final Distance WheelDiameter;

    public final boolean DriveInverted;
    public final boolean TurnInverted;

    public final SwerveDrivetrainConfiguration DriveConfig;

    /**
     * @param angleOffset
     *            Zero offset of this module.
     * @param moduleLocation
     *            Translation from the center of the robot to this module.
     * @param driveGearRatio
     *            Gear ratio between the drive motor and wheel.
     * @param turnGearRatio
     *            Gear ratio between the turning motor and wheel.
     * @param wheelDiameter
     *            Diameter of the wheel.
     * @param driveInverted
     *            Whether or not the drive motor is inverted.
     * @param turnInverted
     *            Whether or not the turning motor is inverted.
     * @param driveConfig
     *            {@link SwerveDrivetrainConfiguration} of the drivetrain this
     *            module is on.
     */
    public SwerveModuleConfiguration(
        Rotation2d angleOffset,
        Translation2d moduleLocation,
        double driveGearRatio,
        double turnGearRatio,
        Distance wheelDiameter,
        boolean driveInverted,
        boolean turnInverted,
        SwerveDrivetrainConfiguration driveConfig) {
        this.DriveConfig = driveConfig;

        this.AngleOffset = angleOffset;
        this.ModuleLocation = moduleLocation;

        this.WheelDiameter = wheelDiameter;

        this.DriveGearRatio = driveGearRatio;
        this.DriveInverted = driveInverted;

        this.TurnGearRatio = turnGearRatio;
        this.TurnInverted = turnInverted;
    }
}
