// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.beaklib.units.Distance;

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

    public static final boolean PRACTICE_CHASSIS = true;

    public static final class PIDConstants {
        public static final class Theta { // TODO: put in drivetrains
            public static final double kP = 6.3;
            public static final double kD = 0.05;
            public static final double[] gains = { kP, 0, kD };
        }
    }

    public static final class DriveConstants {
        public static final double SPEED_SCALE = 0.25;
        public static final double SLOW_SPEED_SCALE = 0.08;
    }

    public static final class ArmConstants {
        public static final double EXTEND_COEFFICIENT = 116.1;
        public static final double RETRACT_COEFFICIENT = 156.58;
        public static final double EXTEND_WAIT_INTERVAL = 0.2;
        public static final double RETRACT_WAIT_INTERVAL = 0.4;
    }

    public static final class FieldConstants {
        public static final Distance FIELD_WIDTH = new Distance(8.0137);
    }
}
