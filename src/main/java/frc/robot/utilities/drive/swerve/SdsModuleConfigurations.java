// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.drive.swerve;

import frc.robot.utilities.drive.swerve.SdsModuleConfiguration.ModuleType;
import frc.robot.utilities.units.Distance;

/** Preset configurations for SDS Swerve Modules. */
public final class SdsModuleConfigurations {
    public static final SdsModuleConfiguration MK2_8p33 = new SdsModuleConfiguration(
            new Distance(0.1016),
            // (16.0 / 40.0) * (24.0 / 20.0) * (15.0 / 60.0),
            (15.0 / 60.0) * (24.0 / 20.0) * (16.0 / 40.0),
            true,
            (1.0 / 18.0), // TODO
            false,
            ModuleType.MK2);
        public static final SdsModuleConfiguration MK2_6p92 = new SdsModuleConfiguration(
            new Distance(0.1016),
            // (16.0 / 40.0) * (24.0 / 20.0) * (15.0 / 60.0),
            (15.0 / 60.0) * (26.0 / 18.0) * (16.0 / 40.0),
            false,
            (1.0 / 18.0), // TODO
            false,
            ModuleType.MK2);
    public static final SdsModuleConfiguration MK3_STANDARD = new SdsModuleConfiguration(
            new Distance(0.1016),
            (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 60.0),
            true,
            (15.0 / 32.0) * (10.0 / 60.0),
            true,
            ModuleType.MK3);
    public static final SdsModuleConfiguration MK3_FAST = new SdsModuleConfiguration(
            new Distance(0.1016),
            (16.0 / 48.0) * (28.0 / 16.0) * (15.0 / 60.0),
            true,
            (15.0 / 32.0) * (10.0 / 60.0),
            true,
            ModuleType.MK3);

    public static final SdsModuleConfiguration MK4_L1 = new SdsModuleConfiguration(
            new Distance(0.10033),
            (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0),
            true,
            (15.0 / 32.0) * (10.0 / 60.0),
            true,
            ModuleType.MK4);
    public static final SdsModuleConfiguration MK4_L2 = new SdsModuleConfiguration(
            new Distance(0.10033),
            (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
            true,
            (15.0 / 32.0) * (10.0 / 60.0),
            true,
            ModuleType.MK4);
    public static final SdsModuleConfiguration MK4_L3 = new SdsModuleConfiguration(
            new Distance(0.10033),
            (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0),
            true,
            (15.0 / 32.0) * (10.0 / 60.0),
            true,
            ModuleType.MK4);
    public static final SdsModuleConfiguration MK4_L4 = new SdsModuleConfiguration(
            new Distance(0.10033),
            (16.0 / 48.0) * (28.0 / 16.0) * (15.0 / 45.0),
            true,
            (15.0 / 32.0) * (10.0 / 60.0),
            true,
            ModuleType.MK4);

    public static final SdsModuleConfiguration MK4I_L1 = new SdsModuleConfiguration(
            new Distance(0.10033),
            (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0),
            true,
            (14.0 / 50.0) * (10.0 / 60.0),
            false,
            ModuleType.MK4i);
    public static final SdsModuleConfiguration MK4I_L2 = new SdsModuleConfiguration(
            new Distance(0.10033),
            (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
            false, // TODO: what
            (150.0 / 7.0),
            true,
            ModuleType.MK4i);
    public static final SdsModuleConfiguration MK4I_L3 = new SdsModuleConfiguration(
            new Distance(0.10033),
            (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0),
            true,
            (14.0 / 50.0) * (10.0 / 60.0),
            false,
            ModuleType.MK4i);
}