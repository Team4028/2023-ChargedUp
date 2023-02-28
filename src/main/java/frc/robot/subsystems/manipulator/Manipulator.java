// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.manipulator;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import edu.wpi.first.wpilibj2.command.Command;

// public class Manipulator extends SubsystemBase {
//     private static Manipulator m_instance;
//     private Gripper m_gripper;
//     private Wrist m_wrist;

//     /** Creates a new Manipulator. */
//     public Manipulator() {
//         // m_gripper = Gripper.getInstance();
//         m_wrist = Wrist.getInstance();
//     }

//     public Command lowCube() {
//         return runOnce(
//             () -> {
//                 m_gripper.runToCubePosition();
//                 m_wrist.runToLowPosition();
//             });
//     }

//     public Command lowCone() {
//         return runOnce(
//             () -> {
//                 m_gripper.runToConePosition();
//                 m_wrist.runToLowPosition();
//             });
//     }

//     public Command midCube() {
//         return runOnce(
//             () -> {
//                 m_gripper.runToCubePosition();
//                 m_wrist.runToMediumPosition();
//             });
//     }

//     public Command midCone() {
//         return runOnce(
//             () -> {
//                 m_gripper.runToConePosition();
//                 m_wrist.runToMediumPosition();
//             });
//     }

//     public Command highCube() {
//         return runOnce(
//             () -> {
//                 m_gripper.runToCubePosition();
//                 m_wrist.runToHighPosition();
//             });
//     }

//     public Command highCone() {
//         return runOnce(
//             () -> {
//                 m_gripper.runToConePosition();
//                 m_wrist.runToHighPosition();
//             });
//     }

//     public static Manipulator getInstance() {
//         if (m_instance == null) {
//             m_instance = new Manipulator();
//         }
//         return m_instance;
//     }

//     @Override
//     public void periodic() {
//         // This method will be called once per scheduler run
//     }
// }
