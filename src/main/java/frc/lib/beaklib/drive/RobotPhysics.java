// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;

/** Class containing various physical parameters of a robot. */
public class RobotPhysics {
    public Measure<Velocity<Distance>> MaxVelocity;
    public Measure<Velocity<Angle>> MaxAngularVelocity;

    public Measure<Velocity<Velocity<Distance>>> MaxAccel;

    public Measure<Distance> TrackWidth;
    public Measure<Distance> WheelBase;

    public Measure<Distance> WheelDiameter;

    public double DriveGearRatio;

    public SimpleMotorFeedforward Feedforward;

    /**
     * Construct a new physics object to pass to a drivetrain.
     * 
     * @param maxVelocity
     *                           Maximum travel velocity of the robot.
     * @param maxAngularVelocity
     *                           Maximum angular velocity of the robot. Set to 0 to
     *                           calculate a
     *                           theoretical.
     * @param maxAccel
     *                           Maximum attainable acceleration of the robot.
     * @param trackWidth
     *                           Distance from the left wheels to the right wheels.
     * @param wheelBase
     *                           Distance from the front wheels to the back wheels.
     * @param wheelDiameter
     *                           Diameter of the wheels.
     * @param driveGearRatio
     *                           Gear ratio of the drive motors.
     * @param feedforward
     *                           A {@link SimpleMotorFeedforward} calculated from
     *                           SysId.
     */
    public RobotPhysics(
            Measure<Velocity<Distance>> maxVelocity,
            Measure<Velocity<Angle>> maxAngularVelocity,
            Measure<Velocity<Velocity<Distance>>> maxAccel,
            Measure<Distance> trackWidth,
            Measure<Distance> wheelBase,
            Measure<Distance> wheelDiameter,
            double driveGearRatio,
            SimpleMotorFeedforward feedforward) {
        this.MaxVelocity = maxVelocity;
        this.MaxAccel = maxAccel;

        this.TrackWidth = trackWidth;
        this.WheelBase = wheelBase;

        this.MaxAngularVelocity = maxAngularVelocity.magnitude() == 0. ? calcTheoreticalAngularVelocity()
                : maxAngularVelocity;

        this.WheelDiameter = wheelDiameter;
        this.DriveGearRatio = driveGearRatio;
        this.Feedforward = feedforward;
    }

    protected Measure<Velocity<Angle>> calcTheoreticalAngularVelocity() {
        Measure<Velocity<Angle>> vel = 
        Units.RadiansPerSecond.of(MaxVelocity.baseUnitMagnitude() /
            Math.hypot(TrackWidth.baseUnitMagnitude() / 2.0, WheelBase.baseUnitMagnitude() / 2.0));
        return vel;
    }
}
