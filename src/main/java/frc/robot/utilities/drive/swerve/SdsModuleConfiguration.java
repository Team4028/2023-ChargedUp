// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.drive.swerve;

import java.util.Objects;

import frc.robot.utilities.units.Distance;

/** Configuration of an SDS Swerve Module. */
public class SdsModuleConfiguration {
    public enum ModuleType {
        MK2,
        MK3,
        MK4,
        MK4i
    }

    public final Distance wheelDiameter;
    public final double driveGearRatio;
    public final boolean driveInverted;

    public final double turnGearRatio;
    public final boolean turnInverted;

    public final ModuleType moduleType;

    /**
     * Creates a new module configuration.
     *
     * @param wheelDiameter  The diameter of the module's wheel in meters.
     * @param driveGearRatio The overall drive reduction of the module. Multiplying
     *                       motor rotations by this value should result in wheel
     *                       rotations.
     * @param driveInverted  Whether the drive motor should be inverted. If there is
     *                       an odd number of gear reductions this is typically
     *                       true.
     * @param turnGearRatio  The overall steer reduction of the module. Multiplying
     *                       motor rotations by this value should result in
     *                       rotations of the steering pulley.
     * @param turnInverted   Whether the steer motor should be inverted. If there is
     *                       an odd number of gear reductions this is typically
     *                       true.
     * @param moduleType     The type of the module (i.e. MK2, MK3, MK4, MK4i)
     */
    public SdsModuleConfiguration(
            Distance wheelDiameter,
            double driveReduction,
            boolean driveInverted,
            double steerReduction,
            boolean steerInverted,
            ModuleType moduleType) {
        this.wheelDiameter = wheelDiameter;
        this.driveGearRatio = driveReduction;
        this.driveInverted = driveInverted;
        this.turnGearRatio = steerReduction;
        this.turnInverted = steerInverted;
        this.moduleType = moduleType;
    }

    /**
     * Gets the diameter of the wheel in meters.
     */
    public Distance getWheelDiameter() {
        return wheelDiameter;
    }

    /**
     * Gets the overall reduction of the drive system.
     * <p>
     * If this value is multiplied by drive motor rotations the result would be
     * drive wheel rotations.
     */
    public double getDriveReduction() {
        return driveGearRatio;
    }

    /**
     * Gets if the drive motor should be inverted.
     */
    public boolean isDriveInverted() {
        return driveInverted;
    }

    /**
     * Gets the overall reduction of the steer system.
     * <p>
     * If this value is multiplied by steering motor rotations the result would be
     * steering pulley rotations.
     */
    public double getSteerReduction() {
        return turnGearRatio;
    }

    /**
     * Gets if the steering motor should be inverted.
     */
    public boolean isSteerInverted() {
        return turnInverted;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o)
            return true;
        if (o == null || getClass() != o.getClass())
            return false;
        SdsModuleConfiguration that = (SdsModuleConfiguration) o;
        return Double.compare(that.getWheelDiameter().getAsMeters(), getWheelDiameter().getAsMeters()) == 0 &&
                Double.compare(that.getDriveReduction(), getDriveReduction()) == 0 &&
                isDriveInverted() == that.isDriveInverted() &&
                Double.compare(that.getSteerReduction(), getSteerReduction()) == 0 &&
                isSteerInverted() == that.isSteerInverted();
    }

    @Override
    public int hashCode() {
        return Objects.hash(
                getWheelDiameter(),
                getDriveReduction(),
                isDriveInverted(),
                getSteerReduction(),
                isSteerInverted());
    }

    @Override
    public String toString() {
        return "ModuleConfiguration{" +
                "wheelDiameter=" + wheelDiameter +
                ", driveReduction=" + driveGearRatio +
                ", driveInverted=" + driveInverted +
                ", steerReduction=" + turnGearRatio +
                ", steerInverted=" + turnInverted +
                '}';
    }
}