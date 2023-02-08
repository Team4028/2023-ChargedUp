// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.units;

import edu.wpi.first.math.util.Units;

/** Class representing an angular velocity in any unit. */
public class AngularVelocity {
    /** Internally represent as radians per second */
    private double m_velocity;

    /**
     * Construct an angular velocity from radians per second.
     */
    public AngularVelocity(double radians) {
        m_velocity = radians;
    }

    /**
     * Construct a zero angular velocity.
     */
    public AngularVelocity() {
        this(0.);
    }

    /**
     * Construct an angular velocity from degrees per second.
     */
    public static AngularVelocity fromDegreesPerSecond(double degrees) {
        return new AngularVelocity(Units.degreesToRadians(degrees));
    }

    /**
     * Construct an angular velocity from rotations per minute.
     */
    public static AngularVelocity fromRotationsPerMinute(double rpm) {
        return new AngularVelocity(Units.rotationsPerMinuteToRadiansPerSecond(rpm));
    }

    /**
     * Construct an angular velocity from rotations per second.
     */
    public static AngularVelocity fromRotationsPerSecond(double rps) {
        return new AngularVelocity(Units.rotationsPerMinuteToRadiansPerSecond(rps * 60.));
    }

    /**
     * Get the angular velocity in radians per second.
     */
    public double getAsRadiansPerSecond() {
        return m_velocity;
    }

    /**
     * Get the angular velocity in degrees per second.
     */
    public double getAsDegreesPerSecond() {
        return Units.radiansToDegrees(m_velocity);
    }

    /**
     * Get the angular velocity in rotations per minute.
     */
    public double getAsRotationsPerMinute() {
        return Units.radiansPerSecondToRotationsPerMinute(m_velocity);
    }

    /**
     * Get the angular velocity in rotations per second.
     */
    public double getAsRotationsPerSecond() {
        return Units.radiansPerSecondToRotationsPerMinute(m_velocity) / 60.;
    }
}
