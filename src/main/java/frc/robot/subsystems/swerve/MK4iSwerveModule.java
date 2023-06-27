// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import frc.lib.beaklib.drive.swerve.BeakSwerveModule;
import frc.lib.beaklib.drive.swerve.SwerveModuleConfiguration;
import frc.lib.beaklib.encoder.BeakCANCoder;
import frc.lib.beaklib.motor.BeakTalonFX;

/** Add your docs here. */
public class MK4iSwerveModule extends BeakSwerveModule {
    public MK4iSwerveModule(
        int driveMotorPort,
        int turnMotorPort,
        int encoderPort,
        SwerveModuleConfiguration config) {
        super(config);

        BeakTalonFX driveMotor = new BeakTalonFX(driveMotorPort, config.DriveConfig.CANBus);
        BeakTalonFX turnMotor = new BeakTalonFX(turnMotorPort, config.DriveConfig.CANBus);
        BeakCANCoder turnEncoder = new BeakCANCoder(encoderPort, config.DriveConfig.CANBus);

        super.setup(driveMotor, turnMotor, turnEncoder);
    }
}
