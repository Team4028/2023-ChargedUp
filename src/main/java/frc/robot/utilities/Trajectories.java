// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import com.pathplanner.lib.path.PathPlannerPath;

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

    // Bruh

    public static PathPlannerPath loadTrajectory(BeakDrivetrain drivetrain, String name,
        double velocityMultiplier, double accelMultiplier) {
        return PathPlannerPath.fromPathFile(name);
            // drivetrain.Physics.maxVelocity.in(MetersPerSecond) * velocityMultiplier,
            // drivetrain.Physics.maxAccel.getAsMetersPerSecondSquared() * accelMultiplier);
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
    public static PathPlannerPath loadPath(BeakDrivetrain drivetrain, PathPosition position, PathPart part,
        String pieceNum, boolean scoreHigh, String data) {
        return loadTrajectory(drivetrain,
            pieceNum + " " + position.name() + " " + part.name() + (data.isEmpty() || data == null ? "" : " " + data),
            position.multiplier(part),
            position.multiplier(part));
    }
}
