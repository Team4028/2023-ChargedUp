// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.lib.beaklib.drive;

// import java.util.Map;
// import java.util.function.Supplier;

// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.commands.PPRamseteCommand;

// import edu.wpi.first.math.controller.RamseteController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
// import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
// import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.wpilibj.RobotBase;
// import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.lib.beaklib.motor.BeakMotorController;

// /** Generic Differential (Tank) Drivetrain subsystem. */
// public class BeakDifferentialDrivetrain extends BeakDrivetrain {
//     public DifferentialDriveKinematics m_kinematics;
//     public DifferentialDriveOdometry m_odom;

//     protected DifferentialDrivetrainSim sim;

//     protected double encoderDistancePerPulse;

//     /**
//      * Construct a new differential drivetrain.
//      * 
//      * @param physics
//      *            A {@link RobotPhysics} object containing the relevant
//      *            information for your robot.
//      * @param thetaPIDGains
//      *            The PID gains for the theta controller.
//      * @param drivePIDGains
//      *            The PID gains for the auton drive controller.
//      * @param generatedDrivePIDGains
//      *            The PID gains for generated paths using a path generation
//      *            command.
//      */
//     public BeakDifferentialDrivetrain(
//         RobotPhysics physics,
//         double[] thetaPIDGains,
//         double[] drivePIDGains,
//         double[] generatedDrivePIDGains,
//         boolean gyroInverted) {
//         super(physics,
//             thetaPIDGains,
//             drivePIDGains,
//             generatedDrivePIDGains,
//             gyroInverted);
//         m_kinematics = new DifferentialDriveKinematics(m_trackWidth.getAsMeters());
//     }

//     /**
//      * Get the drivetrain kinematics.
//      * 
//      * @return {@link DifferentialDriveKinematics} for this drivetrain.
//      */
//     public DifferentialDriveKinematics getKinematics() {
//         return m_kinematics;
//     }

//     // TODO: this needs to be fixed very badly
//     public Command getTrajectoryCommand(PathPlannerTrajectory traj, Map<String, Command> eventMap) {
//         return new PPRamseteCommand(
//             traj,
//             this::getPoseMeters,
//             new RamseteController(),
//             m_feedForward,
//             m_kinematics,
//             this::getWheelSpeeds,
//             m_driveController,
//             m_driveController,
//             this::driveVolts,
//             this).andThen(() -> drive(0, 0, 0));
//     }

//     public Command generatePath(Supplier<Pose2d> desiredPose) {
//         return Commands.none();
//     }

//     /**
//      * Calculate wheel speeds from x and rotation values from joysticks.
//      * 
//      * @param x
//      *            Speed of the robot in the x direction (forward).
//      * @param rot
//      *            Angular rate of the robot.
//      */
//     public DifferentialDriveWheelSpeeds calcWheelSpeeds(
//         double x,
//         double rot) {
//         x *= m_maxVelocity.getAsMetersPerSecond();
//         rot *= m_maxAngularVelocity.getAsRadiansPerSecond();
//         DifferentialDriveWheelSpeeds speed = m_kinematics.toWheelSpeeds(
//             new ChassisSpeeds(x, 0, rot));

//         speed.desaturate(m_maxVelocity.getAsMetersPerSecond());

//         return speed;
//     }

//     /**
//      * Get the current wheel speeds.
//      * 
//      * @return Current wheel speeds of the robot.
//      */
//     public DifferentialDriveWheelSpeeds getWheelSpeeds() {
//         return null;
//     }

//     /**
//      * Get the current wheel speeds.
//      * 
//      * @return Current wheel speeds of the robot.
//      */
//     public DifferentialDriveWheelSpeeds getWheelSpeeds(BeakMotorController frontLeft, BeakMotorController frontRight) {
//         return new DifferentialDriveWheelSpeeds(
//             frontLeft.getRate().Value,
//             frontRight.getRate().Value);
//     }

//     /**
//      * Drive by sending voltage to the motors.
//      * 
//      * @param left
//      *            Volts to send to the left side of the drivetrain.
//      * @param right
//      *            Volts to send to the right side of the drivetrain.
//      */
//     public void driveVolts(double left, double right) {
//     }

//     /**
//      * Get the robot's pose.
//      * 
//      * @return The pose reported from the odometry, measured in meters.
//      */
//     public Pose2d getPoseMeters() {
//         return m_odom.getPoseMeters();
//     }

//     public void resetOdometry(Pose2d pose) {
//         m_odom.resetPosition(getGyroRotation2d(), 0., 0., pose);
//     }

//     public Pose2d updateOdometry(
//         BeakMotorController frontLeftMotorController,
//         BeakMotorController frontRightMotorController) {
//         if (RobotBase.isSimulation()) {
//             sim.setInputs(
//                 frontRightMotorController.getOutputVoltage().Value,
//                 frontLeftMotorController.getOutputVoltage().Value);
//             sim.update(0.02);

//             m_gyroSim.setAngle(-sim.getHeading().getDegrees());
//         }
//         Rotation2d rot = getGyroRotation2d();

//         m_pose = m_odom.update(rot,
//             frontLeftMotorController.getDistance().Value,
//             frontRightMotorController.getDistance().Value);

//         return m_pose;
//     }

//     public boolean isHolonomic() {
//         return false;
//     }
// }