// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.beaklib.units.Acceleration;
import frc.lib.beaklib.units.AngularVelocity;
import frc.lib.beaklib.units.Distance;
import frc.lib.beaklib.units.Velocity;

/** Class containing various physical parameters of a robot. */
public class RobotPhysics {
    public Velocity maxVelocity;
    public AngularVelocity maxAngularVelocity;

    public Acceleration maxAccel;

    public Distance trackWidth;
    public Distance wheelBase;

    public Distance wheelDiameter;

    public double driveGearRatio;

    public SimpleMotorFeedforward feedforward;

    /**
     * Construct a new physics object to pass to a drivetrain.
     * 
     * @param maxVelocity
     *            Maximum travel velocity of the robot.
     * @param maxAngularVelocity
     *            Maximum angular velocity of the robot. Set to 0 to calculate a
     *            theoretical.
     * @param maxAccel
     *            Maximum attainable acceleration of the robot.
     * @param trackWidth
     *            Distance from the left wheels to the right wheels.
     * @param wheelBase
     *            Distance from the front wheels to the back wheels.
     * @param wheelDiameter
     *            Diameter of the wheels.
     * @param driveGearRatio
     *            Gear ratio of the drive motors.
     * @param feedforward
     *            A {@link SimpleMotorFeedforward} calculated from
     *            SysId.
     */
    public RobotPhysics(
        Velocity maxVelocity,
        AngularVelocity maxAngularVelocity,
        Acceleration maxAccel,
        Distance trackWidth,
        Distance wheelBase,
        Distance wheelDiameter,
        double driveGearRatio,
        SimpleMotorFeedforward feedforward) {
        this.maxVelocity = maxVelocity;
        this.maxAccel = maxAccel;

        this.trackWidth = trackWidth;
        this.wheelBase = wheelBase;

        this.maxAngularVelocity = (maxAngularVelocity.getAsRadiansPerSecond() == 0. ? calcTheoreticalAngularVelocity()
            : maxAngularVelocity);

        this.wheelDiameter = wheelDiameter;
        this.driveGearRatio = driveGearRatio;
        this.feedforward = feedforward;
    }

    protected AngularVelocity calcTheoreticalAngularVelocity() {

        AngularVelocity vel = new AngularVelocity(maxVelocity.getAsMetersPerSecond()
            / Math.hypot(trackWidth.getAsMeters() / 2.0, wheelBase.getAsMeters() / 2.0));
        return vel;
    }
}
