package frc.robot.subsystems.arms;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

    @AutoLog
    public static class ArmIOInputs {
        public double armPosition = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double positionError = 0.0;
    }

    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setPosition(double pos) {}

    public default void stop() {}

    public default void setVoltage(double volts) {}
}
