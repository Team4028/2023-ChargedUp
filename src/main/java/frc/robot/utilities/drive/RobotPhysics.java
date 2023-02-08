// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.utilities.units.AngularVelocity;
import frc.robot.utilities.units.Distance;
import frc.robot.utilities.units.Velocity;

/** Class containing various physical parameters of a robot. */
public class RobotPhysics {
    public Velocity maxVelocity;
    public AngularVelocity maxAngularVelocity;

    public Distance trackWidth;
    public Distance wheelBase;

    public Distance wheelDiameter;

    public double driveGearRatio;

    public SimpleMotorFeedforward feedforward;

    /**
     * Construct a new physics object to pass to a drivetrain.
     * 
     * @param maxVelocity        Maximum travel velocity of the robot, in meters per
     *                           second.
     * @param maxAngularVelocity Maximum angular velocity of the robot, in radians
     *                           per second. Set to 0 to calculate a theoretical.
     * @param trackWidth         Distance from the left wheels to the right wheels,
     *                           in inches.
     * @param wheelBase          Distance from the front wheels to the back wheels,
     *                           in inches.
     * @param wheelDiameter      Diameter of the wheels, in inches.
     * @param driveGearRatio     Gear ratio of the drive motors.
     * @param feedforward        A {@link SimpleMotorFeedforward} calculated from
     *                           SysId.
     */
    public RobotPhysics(
            Velocity maxVelocity,
            AngularVelocity maxAngularVelocity,
            Distance trackWidth,
            Distance wheelBase,
            Distance wheelDiameter,
            double driveGearRatio,
            SimpleMotorFeedforward feedforward) {
        this.maxVelocity = maxVelocity;
        this.trackWidth = trackWidth;
        this.wheelBase = wheelBase;
        this.maxAngularVelocity = (maxAngularVelocity.getAsRadiansPerSecond() == 0. ? calcTheoreticalAngularVelocity()
                : maxAngularVelocity);
        this.wheelDiameter = wheelDiameter;
        this.driveGearRatio = driveGearRatio;
        this.feedforward = feedforward;
    }

    protected AngularVelocity calcTheoreticalAngularVelocity() {

        AngularVelocity bruh = new AngularVelocity(maxVelocity.getAsMetersPerSecond()
                / Math.hypot(trackWidth.getAsMeters() / 2.0, wheelBase.getAsMeters() / 2.0));
        System.out.println(bruh.getAsRadiansPerSecond());
        return bruh;
    }
}
