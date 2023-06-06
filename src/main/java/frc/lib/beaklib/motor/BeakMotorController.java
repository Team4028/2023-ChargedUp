// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.motor;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.lib.beaklib.pid.BeakPIDConstants;
import frc.lib.beaklib.units.Distance;
import frc.lib.beaklib.units.Velocity;

/** Common interface for all motor controllers. */
public interface BeakMotorController extends MotorController {
    /**
     * Set the motor to be on brake or coast mode.
     * 
     * @param brake
     *            True = brake, False = coast
     */
    public void setBrake(boolean brake);

    /**
     * Run the motor in velocity mode.
     * </p>
     * 
     * To run in native units, use {@link setVelocityNU}.
     * 
     * @param velocity
     *            Velocity to run.
     * @param arbFeedforward
     *            Arbitrary feed-forward to pass to the motor controller, in volts.
     */
    default void setVelocity(Velocity velocity, double arbFeedforward, int slot) {
        setVelocityNU(
            (velocity.getAsMetersPerSecond() / (getWheelDiameter().getAsMeters() * Math.PI) * 60.) // rpm
                * getEncoderGearRatio() * getVelocityConversionConstant(),
            arbFeedforward, slot);
    }

    /**
     * Run the motor in velocity mode.
     * </p>
     * 
     * To run in native units, use {@link setVelocityNU}.
     * 
     * @param velocity
     *            Velocity to run.
     */
    default void setVelocity(Velocity velocity, int slot) {
        setVelocity(velocity, 0, slot);
    }

    /**
     * Run the motor in velocity mode, in RPM.
     * </p>
     * 
     * To run in native units, use {@link setVelocityNU}.
     * 
     * @param rpm
     *            RPM to run.
     * @param arbFeedforward
     *            Arbitrary feed-forward to pass to the motor controller, in volts.
     */
    default void setVelocityRPM(double rpm, double arbFeedforward, int slot) {
        setVelocityNU(rpm * getVelocityConversionConstant() * getEncoderGearRatio(), arbFeedforward, slot);
    }

    /**
     * Run the motor in velocity mode, in RPM.
     * </p>
     * 
     * To run in native units, use {@link setVelocityNU}.
     * 
     * @param rpm
     *            RPM to run.
     */
    default void setVelocityRPM(double rpm, int slot) {
        setVelocityRPM(rpm, 0, slot);
    }

    /**
     * Run the motor in velocity mode, in NU.
     * </p>
     * NU/100ms for Talons, RPM for SparkMAX.
     * 
     * @param nu
     *            NU to run.
     * @param arbFeedforward
     *            Arbitrary feed-forward to pass to the motor controller, in volts.
     * @param slot
     *            PID slot to run.
     */
    public void setVelocityNU(double nu, double arbFeedforward, int slot);

    /**
     * Run the motor in velocity mode, in NU.
     * </p>
     * NU/100ms for Talons, RPM for SparkMAX.
     * 
     * @param nu
     *            NU to run.
     * @param slot
     *            PID slot to run.
     */
    default void setVelocityNU(double nu, int slot) {
        setVelocityNU(nu, 0, slot);
    }

    /**
     * Run the motor in velocity mode, in NU.
     * </p>
     * NU/100ms for Talons, RPM for SparkMAX.
     * 
     * @param nu
     *            NU to run.
     */
    default void setVelocityNU(double nu) {
        setVelocityNU(nu, 0, 0);
    }

    /**
     * Run the motor in position mode.
     * </p>
     * 
     * To run in native units, use {@link setPositionNU}.
     * 
     * @param distance
     *            Distance to run.
     * @param arbFeedforward
     *            Arbitrary feed-forward to pass to the motor controller, in volts.
     * @param slot
     *            PID slot to run.
     */
    default void setPosition(Distance distance, double arbFeedforward, int slot) {
        setPositionNU(
            (distance.getAsMeters() * getPositionConversionConstant() * getEncoderGearRatio()) //
                / (getWheelDiameter().getAsMeters() * Math.PI),
            arbFeedforward,
            slot);
    }

    /**
     * Run the motor in position mode.
     * </p>
     * 
     * To run in native units, use {@link setPositionNU}.
     * 
     * @param distance
     *            Distance to run.
     * @param slot
     *            PID slot to run.
     */
    default void setPosition(Distance distance, int slot) {
        setPosition(distance, 0, slot);
    }

    /**
     * Run the motor in position mode, in motor rotations.
     * </p>
     * 
     * To run in native units, use {@link setPositionNU}.
     * 
     * @param rotations
     *            Rotations to run.
     * @param arbFeedforward
     *            Arbitrary feed-forward to pass to the motor controller, in volts.
     * @param slot
     *            PID slot to run.
     */
    default void setPositionMotorRotations(double rotations, double arbFeedforward, int slot) {
        setPositionNU(rotations * getPositionConversionConstant() * getEncoderGearRatio(), arbFeedforward, slot);
    }

    /**
     * Run the motor in position mode, in motor rotations.
     * </p>
     * 
     * To run in native units, use {@link setPositionNU}.
     * 
     * @param rotations
     *            Rotations to run.
     * @param slot
     *            PID slot to run.
     */
    default void setPositionMotorRotations(double rotations, int slot) {
        setPositionMotorRotations(rotations, 0, slot);
    }

    /**
     * Run the motor in position mode, in motor rotations.
     * </p>
     * 
     * To run in native units, use {@link setPositionNU}.
     * 
     * @param rotations
     *            Rotations to run.
     */
    default void setPositionMotorRotations(double rotations) {
        setPositionMotorRotations(rotations, 0, 0);
    }

    /**
     * Run the motor in position mode, in NU.
     * </p>
     * 2048 NU per rotation for TalonFX, 4096 for TalonSRX, and usually 1 for
     * SparkMAX.
     * 
     * @param nu
     *            NU to run.
     * @param arbFeedforward
     *            Arbitrary feed-forward to pass to the motor controller, in volts.
     * @param slot
     *            PID slot to run.
     */
    public void setPositionNU(double nu, double arbFeedforward, int slot);

    /**
     * Run the motor in position mode, in NU.
     * </p>
     * 2048 NU per rotation for TalonFX, 4096 for TalonSRX, and usually 1 for
     * SparkMAX.
     * 
     * @param nu
     *            NU to run.
     * @param slot
     *            PID slot to run.
     */
    default void setPositionNU(double nu, int slot) {
        setPositionNU(nu, 0, slot);
    }

    /**
     * Run the motor in position mode, in NU.
     * </p>
     * 2048 NU per rotation for TalonFX, 4096 for TalonSRX, and usually 1 for
     * SparkMAX.
     * 
     * @param nu
     *            NU to run.
     */
    default void setPositionNU(double nu) {
        setPositionNU(nu, 0, 0);
    }

    /**
     * Sets the encoder's position.
     * </p>
     * 
     * To set in native units, use {@link setEncoderPositionNU}.
     * 
     * @param distance
     *            Distance to set the encoder to.
     */
    default void setEncoderPosition(Distance distance) {
        setEncoderPositionNU(
            (distance.getAsMeters() * getPositionConversionConstant() * getEncoderGearRatio()) //
                / (getWheelDiameter().getAsMeters() * Math.PI));
    }

    /**
     * Sets the encoder's position, in motor rotations.
     * </p>
     * 
     * To set in native units, use {@link setEncoderPositionNU}.
     * 
     * @param rotations
     *            Rotations to set the encoder to.
     */
    default void setEncoderPositionMotorRotations(double rotations) {
        setEncoderPositionNU(rotations * getPositionConversionConstant() * getEncoderGearRatio());
    }

    /**
     * Sets the encoder's position, in NU.
     * </p>
     * 2048 NU per rotation for TalonFX, 4096 for TalonSRX, and usually 1 for
     * SparkMAX.
     * 
     * @param nu
     *            NU to set the encoder to.
     */
    public void setEncoderPositionNU(double nu);

    /**
     * Resets the encoder position to 0.
     */
    default void resetEncoder() {
        setEncoderPositionNU(0.);
    }

    /**
     * Run the motor in motion magic mode.
     * </p>
     * 
     * To run in native units, use {@link setMotionMagicNU}.
     * 
     * @param distance
     *            Distance to run.
     * @param arbFeedforward
     *            Arbitrary feed-forward to pass to the motor controller, in volts.
     * @param slot
     *            PID slot to run.
     */
    default void setMotionMagic(Distance distance, double arbFeedforward, int slot) {
        setMotionMagicNU(
            (distance.getAsMeters() * getPositionConversionConstant() * getEncoderGearRatio()) //
                / (getWheelDiameter().getAsMeters() * Math.PI),
            arbFeedforward,
            slot);
    }

    /**
     * Run the motor in motion magic mode.
     * </p>
     * 
     * To run in native units, use {@link setMotionMagicNU}.
     * 
     * @param distance
     *            Distance to run.
     * @param slot
     *            PID slot to run.
     */
    default void setMotionMagic(Distance distance, int slot) {
        setMotionMagic(distance, 0., slot);
    }

    /**
     * Runs the motor in motion magic mode, in motor rotations.
     * </p>
     * 
     * To run in native units, use {@link setMotionMagicNU}.
     * 
     * @param rotations
     *            Rotations to run.
     * @param arbFeedforward
     *            Arbitrary feed-forward to pass to the motor controller, in volts.
     * @param slot
     *            PID slot to run.
     */
    default void setMotionMagicMotorRotations(double rotations, double arbFeedforward, int slot) {
        setMotionMagicNU(rotations * getPositionConversionConstant() * getEncoderGearRatio(), arbFeedforward, slot);
    }

    /**
     * Runs the motor in motion magic mode, in motor rotations.
     * </p>
     * Not currently supported by SparkMAX.
     * 
     * @param rotations
     *            Rotations to run.
     * @param slot
     *            PID slot to run.
     */
    default void setMotionMagicMotorRotations(double rotations, int slot) {
        setMotionMagicMotorRotations(rotations, 0, slot);
    }

    /**
     * Runs the motor in motion magic mode, in motor rotations.
     * </p>
     * Not currently supported by SparkMAX.
     * 
     * @param rotations
     *            Rotations to run.
     */
    default void setMotionMagicMotorRotations(double rotations) {
        setMotionMagicMotorRotations(rotations, 0, 0);
    }

    /**
     * Runs the motor in motion magic mode, in NU.
     * </p>
     * 2048 NU per rotation for TalonFX, 4096 for TalonSRX, and usually 1 for
     * SparkMAX.
     * 
     * @param nu
     *            NU to run.
     * @param arbFeedforward
     *            Arbitrary feed-forward to pass to the motor controller, in volts.
     * @param slot
     *            PID slot to run.
     */
    public void setMotionMagicNU(double nu, double arbFeedforward, int slot);

    /**
     * Runs the motor in motion magic mode, in NU.
     * </p>
     * 2048 NU per rotation for TalonFX, 4096 for TalonSRX, and usually 1 for
     * SparkMAX.
     * 
     * @param nu
     *            NU to run.
     * @param slot
     *            PID slot to run.
     */
    default void setMotionMagicNU(double nu, int slot) {
        setMotionMagicNU(nu, 0, slot);
    }

    /**
     * Runs the motor in motion magic mode, in NU.
     * </p>
     * 2048 NU per rotation for TalonFX, 4096 for TalonSRX, and usually 1 for
     * SparkMAX.
     * 
     * @param nu
     *            NU to run.
     */
    default void setMotionMagicNU(double nu) {
        setMotionMagicNU(nu, 0, 0);
    }

    /**
     * Get the motor velocity.
     * 
     * @return Velocity combined with the timestamp of the received data.
     */
    default DataSignal<Velocity> getSpeed() {
        DataSignal<Double> velocity = getVelocityNU();
        Velocity motorVelocity = new Velocity(velocity.Value * (getWheelDiameter().getAsMeters() * Math.PI)
            / getVelocityConversionConstant() / getEncoderGearRatio());
        return new DataSignal<Velocity>(motorVelocity);
    }

    /**
     * Get the motor velocity, in RPM.
     * 
     * @return Velocity in RPM combined with the timestamp of the received data.
     */
    default DataSignal<Double> getVelocityRPM() {
        DataSignal<Double> velocity = getVelocityNU();
        return new DataSignal<Double>(velocity.Value / getVelocityConversionConstant() / getEncoderGearRatio());
    }

    /**
     * Get the motor velocity, in NU.
     * </p>
     * NU/100ms for Talons, RPM for SparkMAX.
     * 
     * @return Velocity in NU combined with the timestamp of the received data.
     */
    public DataSignal<Double> getVelocityNU();

    /**
     * Get the motor distance.
     * 
     * @param latencyCompensated
     *            Whether or not to attempt latency compensation.
     * 
     * @return Distance combined with the timestamp of the received data.
     */
    default DataSignal<Distance> getDistance(boolean latencyCompensated) {
        DataSignal<Double> position = getPositionNU(latencyCompensated);
        Distance motorDistance = new Distance(position.Value * (getWheelDiameter().getAsMeters() * Math.PI)
            / getPositionConversionConstant() / getEncoderGearRatio());
        return new DataSignal<Distance>(motorDistance, position.Timestamp);
    }

    /**
     * Get the motor position, in motor rotations.
     * 
     * @param latencyCompensated
     *            Whether or not to attempt latency compensation.
     * 
     * @return Position in motor rotations.
     */
    default DataSignal<Double> getPositionMotorRotations(boolean latencyCompensated) {
        DataSignal<Double> position = getPositionNU(latencyCompensated);
        return new DataSignal<Double>(position.Value / getPositionConversionConstant() / getEncoderGearRatio(),
            position.Timestamp);
    }

    /**
     * Get the motor position, in NU.
     * 2048 NU per rotation for TalonFX, 4096 for TalonSRX, and usually 1 for
     * SparkMAX.
     * 
     * @param latencyCompensated
     *            Whether or not to attempt latency compensation.
     * 
     * @return Position in NU combined with the timestamp of the received data.
     */
    public DataSignal<Double> getPositionNU(boolean latencyCompensated);

    /**
     * Stop the motor.
     */
    default void stop() {
        set(0.);
    }

    @Override
    default void disable() {
        stop();
    }

    @Override
    default void stopMotor() {
        stop();
    }

    /**
     * Get the voltage currently being run to the motor controller, with the
     * timestamp of the received data.
     */
    public DataSignal<Double> getSuppliedVoltage();

    /**
     * Get the current applied voltage to the motor controller.
     * 
     * @return Applied voltage.
     */
    default DataSignal<Double> getOutputVoltage() {
        DataSignal<Double> voltage = getSuppliedVoltage();
        voltage.Value *= get();
        return voltage;
    }

    /**
     * Set PIDF gains.
     * 
     * @param constants
     *            PIDF Constants.
     * @param slot
     *            The slot to set these values in.
     */
    public void setPID(
        BeakPIDConstants constants,
        int slot);

    /**
     * Get PIDF gains.
     * 
     * @param slot
     *            The slot to get these values from.
     */
    public BeakPIDConstants getPID(int slot);

    /**
     * Set the reverse limit switch's default state
     * 
     * @param normallyClosed
     *            True if its normal state is "closed", false if its
     *            normal state is "open"
     */
    public void setReverseLimitSwitchNormallyClosed(boolean normallyClosed);

    /**
     * Set the forward limit switch's default state
     * 
     * @param normallyClosed
     *            True if its normal state is "closed", false if its
     *            normal state is "open"
     */
    public void setForwardLimitSwitchNormallyClosed(boolean normallyClosed);

    /**
     * Set the position at which the encoder will be reset to once the reverse limit
     * switch is hit.
     * </p>
     * 
     * Only applies to v6 Talon FX.
     * 
     * @param nu
     *            Position in NU (shaft rotations) to set the encoder to when
     *            hitting the reverse limit switch.
     */
    default void setReverseExtremePosition(double nu) {
    }

    /**
     * Set the position at which the encoder will be reset to once the forward limit
     * switch is hit.
     * </p>
     * 
     * Only applies to v6 Talon FX.
     * 
     * @param nu
     *            Position in NU (shaft rotations) to set the encoder to when
     *            hitting the forward limit switch.
     */
    default void setForwardExtremePosition(double nu) {
    }

    /**
     * Whether or not the limit switch is closed. This is independent of the
     * polarity (normally-closed) option on
     * CTRE devices, but on Spark MAXes, it is dependent--i.e. returning true if the
     * limit switch is not pressed,
     * when it's configured to be normally closed.
     * </p>
     * Also returns the timestamp of the received data.
     */
    public DataSignal<Boolean> getReverseLimitSwitch();

    /**
     * Whether or not the limit switch is closed. This is independent of the
     * polarity (normally-closed) option on
     * CTRE devices, but on Spark MAXes, it is dependent--i.e. returning true if the
     * limit switch is not pressed,
     * when it's configured to be normally closed.
     * </p>
     * Also returns the timestamp of the received data.
     */
    public DataSignal<Boolean> getForwardLimitSwitch();

    /**
     * Set the supply (PDH to controller) current limit.
     * </p>
     * 
     * For Talons, the "tripping" point is set to this plus 5, and the time to trip
     * back to the limit is set to 0.1 seconds.
     * 
     * @param amps
     *            The maximum amps to allow the motor controller to receive.
     */
    public void setSupplyCurrentLimit(int amps);

    /**
     * Set the stator (controller to motor) current limit.
     * </p>
     * 
     * Only supported on TalonFX.
     * </p>
     * 
     * The "tripping" point is set to this plus 5, and the time to trip back to the
     * limit is set to 0.1 seconds.
     * 
     * @param amps
     *            The maximum amps to allow the motor controller to send.
     */
    public void setStatorCurrentLimit(int amps);

    /**
     * Restore the motor controller's factory default settings.
     */
    public void restoreFactoryDefault();

    /**
     * Set the deadband, in NU, where PID control will no longer attempt to respond
     * to an error.
     * 
     * @param error
     *            Error deadband.
     * @param slot
     *            Slot to set to.
     */
    public void setAllowedClosedLoopError(double error, int slot);

    /**
     * Set the voltage compensation saturation for the motor controller.
     * </p>
     * 
     * See CTRE's docs for more info on voltage compensation.
     * </p>
     * 
     * Note: due to the closed-source nature of the motor controller's
     * implementations (JNI), the exact way this works on Spark MAXes and
     * Talons may be inconsistent. For more consistent behavior, use
     * <code>setVoltage</code> instead. This will directly account for
     * voltage drops, with a standardized compensation value on Talons,
     * or directly on the motor controller with PID on the Spark MAX.
     * </p>
     * 
     * For any motor controller, set this to anything greater than 0 to enable it.
     * 
     * @param saturation
     *            Saturation.
     */
    public void setVoltageCompensationSaturation(double saturation);

    /**
     * Set the Motion Magic cruise velocity.
     * </p>
     * 
     * See CTRE's Motion Magic documentation, or REV's Smart Motion example
     * to see what this means.
     * 
     * @param velocity
     *            Cruise velocity, in NU.
     */
    public void setMotionMagicCruiseVelocity(double velocity, int slot);

    /**
     * Set the Motion Magic acceleration.
     * </p>
     * 
     * See CTRE's Motion Magic documentation, or REV's Smart Motion example
     * to see what this means.
     * 
     * @param accel
     *            Acceleration, in NU per second.
     */
    public void setMotionMagicAcceleration(double accel, int slot);

    /**
     * Set the motor controller's speed, in range [-1.0, 1.0], with an arbitrary
     * feed forward.
     * 
     * @param percentOutput
     *            Percent output to pass to the motor controller.
     * @param arbFeedforward
     *            Arbitrary feed-forward to pass to the motor controller, in volts.
     */
    public void set(double percentOutput, double arbFeedforward);

    /**
     * @deprecated This method will no longer be used. Instead, configure motor
     *             controllers directly through the respective vendor's motor
     *             controller API.
     *             </p>
     * 
     *             Set a status frame period of the motor controller.
     *             </p>
     * 
     *             Values are dependent upon the individual motor controller.
     *             Pass a corresponding value for your motor controller (by
     *             appending .value to
     *             the enum), i.e.:
     *             </p>
     * 
     *             <pre>
     * <code>
     * controller.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General.value, 11); // TalonFX
     * sparkController.setStatusFramePeriod(PeriodicFrame.kStatus0.value, 1); // SparkMAX
     * </code>
     *             </pre>
     * 
     * @param value
     *            Value of the status frame, from an enum.
     * @param period
     *            Period of the status frame, in ms.
     */
    @Deprecated(forRemoval = true)
    public void setStatusPeriod(int value, int period);

    /* CONVERSION API */

    /**
     * Set the velocity conversion constant for this motor.
     * </p>
     * 
     * The velocity conversion constant is a factor that, when dividing native
     * velocity units by the constant, outputs rotations per minute.
     * </p>
     * 
     * Default values:
     * <ul>
     * <li>v6 Talon FX, Spark MAX: 1 (NU are RPM)</li>
     * <li>v5 Talon FX: 600 / 2048 (NU/100ms -> RPM).</li>
     * <li>Talon SRX: 600 / 4096 (NU/100ms -> RPM)</li>
     * </ul>
     * 
     * @param constant
     *            Conversion constant. Units: <code>NU/rev/min</code>
     */
    public void setVelocityConversionConstant(double constant);

    /**
     * Get the velocity conversion constant for this motor.
     * </p>
     * 
     * This is used by the RPM and m/s getter/setter methods. Divide the native
     * velocity units by this constant to output rotations per minute.
     * 
     * @return Conversion constant. Units: <code>NU/rev/min</code>
     */
    public double getVelocityConversionConstant();

    /**
     * Set the position conversion constant for this motor.
     * </p>
     * 
     * The position conversion constant is a factor that, when dividing native
     * position units by the constant, outputs rotations.
     * </p>
     * 
     * Default values:
     * <ul>
     * <li>v6 Talon FX, Spark MAX: 1 (NU are rotations)</li>
     * <li>v5 Talon FX: 2048 (NU -> rotations).</li>
     * <li>Talon SRX: 4096 (NU -> rotations)</li>
     * </ul>
     * 
     * @param constant
     *            Conversion constant. Units: <code>NU/rev</code>
     */
    public void setPositionConversionConstant(double constant);

    /**
     * Get the position conversion constant for this motor.
     * </p>
     * 
     * This is used by the rotationsd and meters getter/setter methods. Divide the
     * native
     * position units by this constant to output rotations.
     * 
     * @return Conversion constant. Units: <code>NU/rev</code>
     */
    public double getPositionConversionConstant();

    /**
     * Set the gear ratio between the encoder and output shaft.
     * </p>
     * 
     * This number represents the number of rotations of the motor shaft per
     * rotation of the output shaft. Therefore, if a motor has a 16:1 gearbox
     * attached, this value should be 16.
     * </p>
     * 
     * For motors with integrated encoders, this will generally be greater than 1 if
     * the motor has a gearbox. However, if a non-integrated encoder is mounted
     * after the gearbox, this will be 1.
     * 
     * @param ratio
     *            Gear ratio. Units: coefficient
     */
    public void setEncoderGearRatio(double ratio);

    /**
     * Get the gear ratio between the encoder and output shaft.
     * </p>
     * 
     * This number represents the number of rotations of the motor shaft per
     * rotation of the output shaft. Therefore, if a motor has a 16:1 gearbox
     * attached, this value should be 16.
     * </p>
     * 
     * Divide the motor rotations or RPM by this number to get the actual rotations
     * or RPM of the final output shaft. Multiply rotations of the output shaft by
     * this number to get the number of motor rotations.
     * 
     * @return Gear ratio. Units: coefficient
     */
    public double getEncoderGearRatio();

    /**
     * Set the diameter of the wheel driven by this motor.
     * </p>
     * 
     * If the motor does not drive a traditional wheel but instead operates a linear
     * actuation mechanism, set this to the diameter of whatever circular object it
     * is rotating.
     * 
     * @param diameter
     *            Diameter of the wheel. Units: distance
     */
    public void setWheelDiameter(Distance diameter);

    /**
     * Get the diameter of the wheel driven by this motor.
     * </p>
     * 
     * Multiply the number of motor rotations or RPM by this number to get the
     * distance travelled by this motor, or the linear speed of the wheel attached
     * to it. Divide the speed or distance by this number to get output shaft
     * rotations.
     * </p>
     * 
     * Note that multiplying RPM by this will net meters per minute, so to get
     * meters per second, you need to divide by 60.
     * 
     * @return Diameter of the wheel. Units: distance
     */
    public Distance getWheelDiameter();
}
