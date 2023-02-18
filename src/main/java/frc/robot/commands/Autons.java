// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auton.BeakAutonCommand;
import frc.robot.commands.chassis.AddVisionMeasurement;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.arms.LowerArm;
import frc.robot.utilities.drive.BeakDrivetrain;
import frc.robot.utilities.drive.Trajectories;

/** Stores all autonomous routines and helper functions. */
public class Autons {
    // Global Subsystems
    private final BeakDrivetrain m_drivetrain;
    private final LowerArm m_lowerArm;
    private final Vision m_aprilTagVision;
    // private final Vision m_gamePieceVision;

    private final Map<String, Command> m_eventMap;

    public Autons(
        BeakDrivetrain drivetrain,
        LowerArm lowerArm,
        Vision aprilTagVision) {
        // Vision gamePieceVision) {
        m_drivetrain = drivetrain;
        m_lowerArm = lowerArm;
        m_aprilTagVision = aprilTagVision;
        // m_gamePieceVision = gamePieceVision;

        m_eventMap = new HashMap<String, Command>();
        m_eventMap.put("ArmScoring", new RunArm(45., m_lowerArm));
    }

    public BeakAutonCommand TwoPieceTopAcquire() {
        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, Trajectories.TwoPieceAcquirePiece(m_drivetrain),
            new AddVisionMeasurement(m_drivetrain, m_aprilTagVision),
            new RunArm(45., m_lowerArm),
            new WaitCommand(0.2),
            new RunArm(12., m_lowerArm).alongWith(
                m_drivetrain.getTrajectoryCommand(Trajectories.TwoPieceAcquirePiece(m_drivetrain), m_eventMap)
            //
            )
        //
        );

        return cmd;
    }

    public BeakAutonCommand TwoPieceTopScore() {
        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, Trajectories.TwoPieceScorePiece(m_drivetrain),
            // new AddVisionMeasurement(m_drivetrain, m_aprilTagVision),
            new RunArm(12., m_lowerArm),
            m_drivetrain.getTrajectoryCommand(Trajectories.TwoPieceScorePiece(m_drivetrain), m_eventMap)
                .deadlineWith(
                    new RepeatCommand(new AddVisionMeasurement(m_drivetrain, m_aprilTagVision))
                //
                )
        //
        );

        return cmd;
    }

    public BeakAutonCommand TwoPieceTop() {
        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, Trajectories.TwoPieceAcquirePiece(m_drivetrain),
            TwoPieceTopAcquire(),
            TwoPieceTopScore());

        return cmd;
    }

    public BeakAutonCommand JPath1() {
        BeakAutonCommand cmd = new BeakAutonCommand(m_drivetrain, Trajectories.JPath1(m_drivetrain),
            m_drivetrain.getTrajectoryCommand(Trajectories.JPath1(m_drivetrain), m_eventMap),
            new WaitCommand(0.1),
            m_drivetrain.generatePath(() -> m_aprilTagVision.getTargetPose(m_drivetrain.getPoseMeters(),
                new Transform3d(new Translation3d(Units.inchesToMeters(54.),
                    Units.inchesToMeters(-0.), 0.),
                    new Rotation3d()))) // 54 inches away from target
        );

        return cmd;
    }
}
