// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.drive;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

/** Get auton trajectories from paths. */
public class Trajectories {
    public static PathPlannerTrajectory JPath1(BeakDrivetrain drivetrain) {
        return PathPlanner.loadPath("j path 1",
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * 0.5,
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * 0.5);
    }

    public static PathPlannerTrajectory JPath2(BeakDrivetrain drivetrain) {
        return PathPlanner.loadPath("j path 2",
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * 0.5,
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * 0.5);
    }

    public static PathPlannerTrajectory TwoPieceTopBalance(BeakDrivetrain drivetrain) {
        return PathPlanner.loadPath("2 Piece TopBalance",
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * 0.25,
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * 0.25);
    }

    public static PathPlannerTrajectory TwoPieceTopAcquirePiece(BeakDrivetrain drivetrain) {
        return PathPlanner.loadPath("2 Piece Top Acquire Piece",
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * 0.5,
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * 0.5);
    }

    public static PathPlannerTrajectory TwoPieceTopScorePiece(BeakDrivetrain drivetrain) {
        return PathPlanner.loadPath("2 Piece Top Score Piece",
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * 0.5,
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * 0.5);
    }

    public static PathPlannerTrajectory TwoPieceBottomAcquirePiece(BeakDrivetrain drivetrain) {
        return PathPlanner.loadPath("2 Piece Bottom Acquire Piece",
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * 0.45,
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * 0.45);
    }

    public static PathPlannerTrajectory TwoPieceBottomScorePiece(BeakDrivetrain drivetrain) {
        return PathPlanner.loadPath("2 Piece Bottom Score Piece",
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * 0.45,
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * 0.45);
    }

    // public static Trajectory getTrajectory(String path) {
    // Trajectory traj = new Trajectory();
    // try {
    // Path trajPath = Filesystem.getDeployDirectory().toPath().resolve("output/" +
    // path + ".wpilib.json");
    // traj = TrajectoryUtil.fromPathweaverJson(trajPath);
    // } catch (IOException e) {
    // System.out.println("Failed to load path " + path);
    // }

    // return traj;
    // }
}
