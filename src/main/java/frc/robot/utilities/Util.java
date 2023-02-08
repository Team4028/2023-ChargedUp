// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

/** Useful utility functions. */
public final class Util {
    public static final double speedScale(double input, double base, double throttle) {
        return input * (base + throttle * (1.0 - base));
    }
}
