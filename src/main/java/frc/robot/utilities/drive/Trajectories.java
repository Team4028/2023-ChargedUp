// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.drive;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

/** Get auton trajectories from paths. */
public class Trajectories {
    public static PathPlannerTrajectory TestPath(BeakDrivetrain drivetrain) {
        return PathPlanner.loadPath("TestPath",
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond(),
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond());
    }

    public static PathPlannerTrajectory NewTestPath(BeakDrivetrain drivetrain) {
        return PathPlanner.loadPath("NewTestPath",
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * 0.25, // velocity
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * 0.25); // acceleration);
    }

    public static PathPlannerTrajectory Ball1(BeakDrivetrain drivetrain) {
        return PathPlanner.loadPath("Ball1",
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond(),
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond());
    }

    public static PathPlannerTrajectory Ball2(BeakDrivetrain drivetrain) {
        return PathPlanner.loadPath("Ball2",
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond(),
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond());
    }

    public static PathPlannerTrajectory CarsonVPath(BeakDrivetrain drivetrain) {
        return PathPlanner.loadPath("Carson V Path",
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond(),
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond());
    }

    public static PathPlannerTrajectory SamPath(BeakDrivetrain drivetrain) {
        return PathPlanner.loadPath("Sam Path",
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond(),
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond());
    }
    
    public static PathPlannerTrajectory NickPath(BeakDrivetrain drivetrain) {
        return PathPlanner.loadPath("Nick Path",
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond(),
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond());
    }
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

    public static PathPlannerTrajectory TwoPieceDriveUp(BeakDrivetrain drivetrain) {
        return PathPlanner.loadPath("2 Piece Drive Up",
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * 0.25,
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * 0.25);
    }

    public static PathPlannerTrajectory TwoPieceAcquirePiece(BeakDrivetrain drivetrain) {
        return PathPlanner.loadPath("2 Piece Acquire Piece",
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * 0.75,
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * 0.75);
    }

    public static PathPlannerTrajectory TwoPieceScorePiece(BeakDrivetrain drivetrain) {
        return PathPlanner.loadPath("2 Piece Score Piece",
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * 0.75,
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * 0.75);
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
