// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.units;

import edu.wpi.first.math.util.Units;

/** Class representing an acceleration in any unit. */
public class Acceleration {
    /** Internally represent as meters per second squared. */
    private double m_accel;

    /**
     * Create an acceleration from a meters per second squared value.
     */
    public Acceleration(double meters) {
        m_accel = meters;
    }

    /**
     * Construct a zero acceleration.
     */
    public Acceleration() {
        this(0.);
    }

    /**
     * Construct an acceleration from an inches per second squared value.
     */
    public static Acceleration fromInchesPerSecondSquared(double inches) {
        return new Acceleration(Units.inchesToMeters(inches));
    }

    /**
     * Construct an acceleration from a feet per second squared value.
     */
    public static Acceleration fromFeetPerSecondSquared(double feet) {
        return new Acceleration(Units.feetToMeters(feet));
    }

    /**
     * Construct an acceleration from a centimeters per second squared value.
     */
    public static Acceleration fromCentimetersPerSecondSquared(double cm) {
        return new Acceleration(cm / 100.);
    }

    /**
     * Construct an acceleration from a miles per hour per second value.
     */
    public static Acceleration fromMilesPerHourPerSecond(double mph) {
        /* Converts to feet per second first. */
        return new Acceleration(Units.feetToMeters(mph * 5280. / 3600.));
    }

    /**
     * Get the acceleration in meters per second squared.
     */
    public double getAsMetersPerSecondSquared() {
        return m_accel;
    }

    /**
     * Get the acceleration in inches per second squared.
     */
    public double getAsInchesPerSecondSquared() {
        return Units.metersToInches(m_accel);
    }

    /**
     * Get the acceleration in feet per second squared.
     */
    public double getAsFeetPerSecondSquared() {
        return Units.metersToFeet(m_accel);
    }

    /**
     * Get the acceleration in centimeters per second squared.
     */
    public double getAsCentimetersPerSecondSquared() {
        return m_accel * 100.;
    }

    /**
     * Get the acceleration in miles per hour per second.
     */
    public double getAsMilesPerHourPerSecond() {
        /* Convert to feet per hour first. */
        return Units.metersToFeet(m_accel * 3600.) / 5280.;
    }
}
