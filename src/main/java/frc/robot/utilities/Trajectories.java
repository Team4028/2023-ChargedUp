// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.lib.beaklib.drive.BeakDrivetrain;

/** Get auton trajectories from paths. */
public class Trajectories {
    /**
     * The "position" of a path, classified as bottom, middle, or top.
     *
     * <p>
     * This enum also stores data for trajectory loading.
     */
    public enum PathPosition {
        BOTTOM("Bottom", 0.25, 0.4, 0.25), // Over the cable cover
        MIDDLE("Middle", 0.5, 0.5, 0.25), // Over the charge station
        // TOP("Top", 0.6, 0.65, 0.25); // Short side
        TOP("Top", 0.2, 0.2, 0.25); // Short side

        /**
         * The name of the position, used in loading trajectories.
         */
        public String name;

        /**
         * What to multiply the max speed and acceleration of paths that acquire pieces
         * by.
         */
        public double acquireMultiplier;

        /**
         * What to multiply the max speed and acceleration of paths that score pieces
         * by.
         */
        public double scoreMultiplier;

        /**
         * What to multiply the max speed and acceleration of paths that balance on the
         * charge station by.
         */
        public double balanceMultiplier;

        private PathPosition(String name, double acquireMultiplier, double scoreMultiplier, double balanceMultiplier) {
            this.name = name;
            this.acquireMultiplier = acquireMultiplier;
            this.scoreMultiplier = scoreMultiplier;
            this.balanceMultiplier = balanceMultiplier;
        }
    }

    // Example paths
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

    public static PathPlannerTrajectory OnePieceBalance(BeakDrivetrain drivetrain, PathPosition position) {
        // All autons are named with the same scheme. We multiply the max speed of the
        // drivetrain by the position's desired multiplier to ensure that each position
        // can run at the necessary speed.

        return PathPlanner.loadPath("1 Piece " + position.name + " Balance",
            drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * position.balanceMultiplier,
            drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * position.balanceMultiplier);
    }

    public static PathPlannerTrajectory TwoPieceBalance(BeakDrivetrain drivetrain, PathPosition position) {
        return PathPlanner.loadPath("2 Piece " + position.name + " Balance",
            drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * position.balanceMultiplier,
            drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * position.balanceMultiplier);
    }

    public static PathPlannerTrajectory TwoPieceAcquirePiece(BeakDrivetrain drivetrain, PathPosition position) {
        return PathPlanner.loadPath("2 Piece " + position.name + " Acquire Piece",
            drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * position.acquireMultiplier,
            drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * position.acquireMultiplier);
    }

    public static PathPlannerTrajectory TwoPieceScorePiece(BeakDrivetrain drivetrain, PathPosition position) {
        return PathPlanner.loadPath("2 Piece " + position.name + " Score Piece",
            drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * position.scoreMultiplier,
            drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * position.scoreMultiplier);
    }

    public static PathPlannerTrajectory ThreePieceAcquirePiece(BeakDrivetrain drivetrain, PathPosition position) {
        return PathPlanner.loadPath("3 Piece " + position.name + " Acquire Piece",
            drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * position.acquireMultiplier,
            drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * position.acquireMultiplier);
    }

    public static PathPlannerTrajectory ThreePieceScorePiece(BeakDrivetrain drivetrain, PathPosition position) {
        return PathPlanner.loadPath("3 Piece " + position.name + " Score Piece",
            drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * position.scoreMultiplier,
            drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * position.scoreMultiplier);
    }
}
