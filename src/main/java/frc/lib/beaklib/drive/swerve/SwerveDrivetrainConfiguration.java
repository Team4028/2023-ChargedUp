// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.drive.swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.beaklib.drive.RobotPhysics;
import frc.lib.beaklib.pid.BeakPIDConstants;

/** Constants that are common among all swerve modules in a drivetrain. */
public class SwerveDrivetrainConfiguration {
    public final BeakPIDConstants DrivePID;
    public final BeakPIDConstants TurnPID;

    public final double AllowedClosedLoopError;

    public final int TurnCurrentLimit;
    public final int DriveSupplyLimit;
    public final int DriveStatorLimit;

    public final String CANBus;
    public final SimpleMotorFeedforward Feedforward;

    public final boolean IsOpenLoop;

    public final RobotPhysics Physics;

    /**
     * Create a new Swerve Drivetrain Configuration.
     * 
     * @param drivePID
     *            PID constants for the drive motor, if applicable.
     * @param turnPID
     *            PID constants for the turning motor.
     * @param isOpenLoop
     *            Set to true if your drivetrain is not characterized.
     * @param allowedClosedLoopError
     *            Allowed error of the turning motor, in NU.
     * @param turnCurrentLimit
     *            Current limit of the turning motor.
     * @param driveSupplyLimit
     *            Supply current limit of the drive motor.
     * @param driveStatorLimit
     *            Stator current limit of the drive motor (if on
     *            Falcons).
     * @param CANBus
     *            CAN Bus that the drivetrain lies on (leave
     *            blank for default).
     * @param feedforward
     *            {@link SimpleMotorFeedforward} for the
     *            drivetrain.
     * @param physics
     *            {@link RobotPhysics} of the drivetrain.
     */
    public SwerveDrivetrainConfiguration(
        BeakPIDConstants drivePID,
        BeakPIDConstants turnPID,
        boolean isOpenLoop,
        double allowedClosedLoopError,
        int turnCurrentLimit,
        int driveSupplyLimit,
        int driveStatorLimit,
        String CANBus,
        SimpleMotorFeedforward feedforward,
        RobotPhysics physics) {
        this.DrivePID = drivePID;
        this.TurnPID = turnPID;
        this.IsOpenLoop = isOpenLoop;
        this.AllowedClosedLoopError = allowedClosedLoopError;
        this.TurnCurrentLimit = turnCurrentLimit;
        this.DriveSupplyLimit = driveSupplyLimit;
        this.DriveStatorLimit = driveStatorLimit;
        this.CANBus = CANBus;
        this.Feedforward = feedforward;
        this.Physics = physics;
    }
}
