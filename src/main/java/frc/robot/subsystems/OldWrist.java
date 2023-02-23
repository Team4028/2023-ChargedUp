// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxPIDController;
// import com.revrobotics.CANSparkMax.ControlType;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.utilities.motor.BeakSparkMAX;

// public class OldWrist extends SubsystemBase {
//     public enum WristPositions {
//         STOW(0.0),
//         INFEED_CUBE(0.0), // TODO: FILL IN COOEFS. Also the different angles for INFEED_CUBE vs.
//                           // INFEED_CONE will become redundant after the installation of the wildstang
//                           // infeed
//         INFEED_CONE(0.0),
//         SCORE_HIGH(0.0),
//         SCORE_MID(0.0);

//         public double position;

//         private WristPositions(double position) {
//             this.position = position;
//         }
//     }

//     private RelativeEncoder m_encoder;
//     private SparkMaxPIDController m_pid;
//     private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
//     private BeakSparkMAX m_motor;
//     private static OldWrist m_instance;
//     private double m_pidPos;

//     /** Creates a new Wrist. */
//     public OldWrist() {
//         m_motor = new BeakSparkMAX(12);
//         m_encoder = m_motor.getEncoder();
//         m_pid = m_motor.getPIDController();
//         // PID Constants
//         kP = 0;
//         kI = 0;
//         kD = 0;
//         kIz = 0;
//         kFF = 0;
//         kMaxOutput = 0.9;
//         kMinOutput = -0.9;
//         m_pid.setP(kP);
//         m_pid.setI(kI);
//         m_pid.setD(kD);
//         m_pid.setIZone(kIz);
//         m_pid.setFF(kFF);
//         m_pid.setOutputRange(kMinOutput, kMaxOutput);
//     }

//     public Command runWrist(double speed) {
//         return runOnce(() -> {
//             m_motor.set(speed);
//         });
//     }

//     public void runWristToPosition(double position) {
//         m_pid.setReference(position, ControlType.kPosition);
//         m_pidPos = position;
//     }

//     public double getTargetPosition() {
//         return m_pidPos;
//     }

//     public double getEncoderPosition() {
//         return m_encoder.getPosition();
//     }

//     public static Wrist getInstance() {
//         if (m_instance == null) {
//             m_instance = new OldWrist();
//         }
//         return m_instance;
//     }

//     @Override
//     public void periodic() {
//         // This method will be called once per scheduler run
//         SmartDashboard.putNumber("Wrist Encoder", m_encoder.getPosition());
//     }
// }
