// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import frc.lib.beaklib.drive.swerve.BeakSwerveModule;
import frc.lib.beaklib.drive.swerve.SwerveModuleConfiguration;
import frc.lib.beaklib.encoder.BeakV6CANCoder;
import frc.lib.beaklib.motor.BeakV6TalonFX;

/** Add your docs here. */
public class MK4iSwerveModule extends BeakSwerveModule {
    public MK4iSwerveModule(
        int driveMotorPort,
        int turnMotorPort,
        int encoderPort,
        SwerveModuleConfiguration config) {
        super(config);

        BeakV6TalonFX driveMotor = new BeakV6TalonFX(driveMotorPort, config.DriveConfig.CANBus);
        BeakV6TalonFX turnMotor = new BeakV6TalonFX(turnMotorPort, config.DriveConfig.CANBus);
        BeakV6CANCoder turnEncoder = new BeakV6CANCoder(encoderPort, config.DriveConfig.CANBus);

        super.setup(driveMotor, turnMotor, turnEncoder);
    }
}
