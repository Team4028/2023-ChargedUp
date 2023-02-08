// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.drive.swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/** Constants that are common among all swerve modules in a drivetrain. */
public class SwerveDrivetrainConfiguration {
    public final double drive_kP;
    public final double turn_kP;
    public final double turn_kD;

    public final double allowedClosedLoopError;

    public final int turnCurrentLimit;
    public final int driveSupplyLimit;
    public final int driveStatorLimit;

    public final String CANBus;
    public final SimpleMotorFeedforward feedforward;

    public final SdsModuleConfiguration moduleConfiguration;

    /**
     * Create a new Swerve Drivetrain Configuration.
     * 
     * @param drive_kP               Proportional gain to use for the drive motor.
     * @param turn_kP                Proportional gain to use for the turning motor.
     * @param turn_kD               Derivative gain to use for the turning motor.
     * @param allowedClosedLoopError Allowed error of the turning motor, in NU.
     * @param turnCurrentLimit       Current limit of the turning motor.
     * @param driveSupplyLimit       Supply current limit of the drive motor.
     * @param driveStatorLimit       Stator current limit of the drive motor (if on
     *                               Falcons).
     * @param CANBus                 CAN Bus that the drivetrain lies on (leave
     *                               blank for default).
     * @param feedforward            {@link SimpleMotorFeedforward} for the
     *                               drivetrain.
     * @param moduleConfiguration    {@link SdsModuleConfiguration} of the modules
     *                               on the drivetrain.
     */
    public SwerveDrivetrainConfiguration(
            double drive_kP,
            double turn_kP,
            double turn_kD,
            double allowedClosedLoopError,
            int turnCurrentLimit,
            int driveSupplyLimit,
            int driveStatorLimit,
            String CANBus,
            SimpleMotorFeedforward feedforward,
            SdsModuleConfiguration moduleConfiguration) {
        this.drive_kP = drive_kP;
        this.turn_kP = turn_kP;
        this.turn_kD = turn_kD;
        this.allowedClosedLoopError = allowedClosedLoopError;
        this.turnCurrentLimit = turnCurrentLimit;
        this.driveSupplyLimit = driveSupplyLimit;
        this.driveStatorLimit = driveStatorLimit;
        this.CANBus = CANBus;
        this.feedforward = feedforward;
        this.moduleConfiguration = moduleConfiguration;
    }
}
