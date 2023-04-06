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
        Bottom(0.45, 0.45, 0.75), // Over the cable cover
        Middle(0.5, 0.5, 0.75), // Over the charge station
        Top(0.6, 0.6, 0.75); // Short side

        /**
         * What to multiply the max speed of paths that acquire pieces by.
         */
        public double acquireMultiplier;

        /**
         * What to multiply the max speed of paths that score pieces by.
         */
        public double scoreMultiplier;

        /**
         * What to multiply the max speed of paths that balance on the charge station
         * by.
         */
        public double balanceMultiplier;

        private PathPosition(double acquireMultiplier, double scoreMultiplier, double balanceMultiplier) {
            this.acquireMultiplier = acquireMultiplier;
            this.scoreMultiplier = scoreMultiplier;
            this.balanceMultiplier = balanceMultiplier;
        }

        public double multiplier(PathPart part) {
            switch (part) {
                case Bal:
                    return balanceMultiplier;
                case Acquire:
                case Park:
                    return acquireMultiplier;
                case Score:
                    return scoreMultiplier;
                default:
                    return acquireMultiplier;
            }
        }
    }

    public enum PathPart {
        Bal, Acquire, Score, Park
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

    // Since all autons use the same naming scheme, we can use a generic function to
    // load all paths.

    /**
     * Load an auton path.
     * 
     * @param drivetrain
     *            The drivetrain to use.
     * @param position
     *            What position this path is in (flat, bump, station)
     * @param part
     *            What part of the auton this is (acquire, score, balance, park)
     * @param pieceNum
     *            What piece this path is acquiring or scoring (for balance, the
     *            last piece scored)
     * @param scoreHigh
     *            Whether to score the preloaded game piece high or low.
     * @param data
     *            Any additional path data that may or may not be present.
     * 
     * @return The final trajectory.
     */
    public static PathPlannerTrajectory loadPath(BeakDrivetrain drivetrain, PathPosition position, PathPart part,
        String pieceNum, boolean scoreHigh, String data) {
        return PathPlanner.loadPath(
            pieceNum + " " + position.name() + " " + part.name() + (data.isEmpty() || data == null ? "" : " " + data),
            drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * position.multiplier(part),
            drivetrain.getPhysics().maxAccel.getAsMetersPerSecondSquared());
    }

    // public static PathPlannerTrajectory OnePieceBalance(BeakDrivetrain drivetrain, PathPosition position) {
    //     // All autons are named with the same scheme. We multiply the max speed of the
    //     // drivetrain by the position's desired multiplier to ensure that each position
    //     // can run at the necessary speed.

    //     return PathPlanner.loadPath("1 " + position.name() + " Bal",
    //         drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * position.balanceMultiplier,
    //         drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * position.balanceMultiplier);
    // }

    // public static PathPlannerTrajectory TwoPieceBalance(BeakDrivetrain drivetrain, PathPosition position) {
    //     return PathPlanner.loadPath("2 Piece " + position.name() + " Balance",
    //         drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * position.balanceMultiplier,
    //         drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * position.balanceMultiplier);
    // }

    // public static PathPlannerTrajectory TwoPieceAcquirePiece(BeakDrivetrain drivetrain, PathPosition position) {
    //     return PathPlanner.loadPath("2 Piece " + position.name() + " Acquire Piece",
    //         drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * position.acquireMultiplier,
    //         drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * position.acquireMultiplier);
    // }

    // public static PathPlannerTrajectory TwoPieceScorePiece(BeakDrivetrain drivetrain, PathPosition position) {
    //     return PathPlanner.loadPath("2 Piece " + position.name() + " Score Piece",
    //         drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * position.scoreMultiplier,
    //         drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * position.scoreMultiplier);
    // }

    // public static PathPlannerTrajectory ThreePieceAcquirePiece(BeakDrivetrain drivetrain, PathPosition position) {
    //     return PathPlanner.loadPath("3 Piece " + position.name() + " Acquire Piece",
    //         drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * position.acquireMultiplier,
    //         drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * position.acquireMultiplier);
    // }

    // public static PathPlannerTrajectory ThreePieceScorePiece(BeakDrivetrain drivetrain, PathPosition position) {
    //     return PathPlanner.loadPath("3 Piece " + position.name() + " Score Piece",
    //         drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * position.scoreMultiplier,
    //         drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * position.scoreMultiplier);
    // }

    // public static PathPlannerTrajectory TwoQuarterPiecePark(BeakDrivetrain drivetrain, PathPosition position) {
    //     return PathPlanner.loadPath("2.25 Piece " + position.name() + " Park",
    //         drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * position.acquireMultiplier,
    //         drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * position.acquireMultiplier);
    // }
}
