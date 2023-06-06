// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.drive.swerve;

import frc.lib.beaklib.drive.swerve.SdsModuleConfiguration.ModuleType;
import frc.lib.beaklib.units.Distance;

/** Preset configurations for SDS Swerve Modules. */
public final class SdsModuleConfigurations {
    public static final SdsModuleConfiguration MK2_8p33 = new SdsModuleConfiguration(
        new Distance(0.1016),
        // (16.0 / 40.0) * (24.0 / 20.0) * (15.0 / 60.0),
        1 / ((15.0 / 60.0) * (24.0 / 20.0) * (16.0 / 40.0)),
        true,
        18.0, // TODO
        false,
        ModuleType.MK2);
    public static final SdsModuleConfiguration MK2_6p92 = new SdsModuleConfiguration(
        new Distance(0.1016),
        // (16.0 / 40.0) * (24.0 / 20.0) * (15.0 / 60.0),
        (15.0 / 60.0) * (26.0 / 18.0) * (16.0 / 40.0),
        false,
        18.0, // TODO
        false,
        ModuleType.MK2);
    public static final SdsModuleConfiguration MK4I_L2 = new SdsModuleConfiguration(
        Distance.fromInches(3.82), // new Distance(0.10033),
        1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)),
        false, // TODO: what
        (150.0 / 7.0),
        true,
        ModuleType.MK4i);
}