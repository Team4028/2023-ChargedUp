// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final Mode currentMode = Mode.REAL;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final class PIDConstants {
        public static final class Theta { // TODO: put in drivetrains
            public static final double kP = 6.3;
            public static final double kD = 0.05;
            public static final double[] gains = { kP, 0, kD };
        }
    }

    public static final class DriveConstants {
        public static final double SPEED_SCALE = 0.25;
    }

    public static final class ArmConstants {
        public static final double EXTEND_COEFFICIENT = 116.1;
        public static final double RETRACT_COEFFICIENT = 156.58;
        public static final double EXTEND_WAIT_INTERVAL = 0.2;
        public static final double RETRACT_WAIT_INTERVAL = 0.4;

        public static final class LowerArmConstants{
            public static final double kP = 0.1;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kIz = 0.0;
            public static final double kFF = 0.0;
            public static final double kMaxOutput = 0.5;
            public static final double kMinOutput = -0.5;
            public static final double kS = 0.33069;
            public static final double kG = 0.2554;
            public static final double kV = 0.10667;

        }

        public static final class UpperArmConstants{
            public static final double kP = 0.05;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kIz = 0.0;
            public static final double kFF = 0.0;
            public static final double kMaxOutput = 0.5;
            public static final double kMinOutput = -0.5;
            public static final double kS = 0.33069;
            public static final double kG = 0.2554;
            public static final double kV = 0.10667;
        }

    }
        public enum ScoringPositions {
            STOWED(1, 1.5, 300.0), // L: 2. U: 2.
            INTERMEDIATE_LOW(3,3,250),
            SCORE_MID(13., 3., 50.), // L: 44. U: 52.
            SCORE_HIGH(13., 19., 50.), // L: 56. U: 80.
            ACQUIRE_FLOOR_CUBE(1.5, 14., 180.), // L: 9. U: 40.
            ACQUIRE_FLOOR_TIPPED_CONE(1.5, 14., 180.), //L: 9. U: 40. 
            ACQUIRE_FLOOR_UPRIGHT_CONE(2.25, 13, 180);
    
            public double lowerPosition;
            public double upperPosition;
            public double wristAngle;
    
            private ScoringPositions(double lowerPosition, double upperPosition, double wristAngle) {
                this.lowerPosition = lowerPosition;
                this.upperPosition = upperPosition;
                this.wristAngle=wristAngle;
            }
        }
}
