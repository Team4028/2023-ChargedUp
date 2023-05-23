// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.motor;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.lib.beaklib.pid.BeakPIDConstants;
import frc.lib.beaklib.units.Distance;

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
     * Run the motor in velocity mode, in RPM.
     * 
     * @param rpm
     *            RPM to run.
     * @param arbFeedforward
     *            Arbitrary feed-forward to pass to the motor controller, in volts.
     */
    public void setVelocityRPM(double rpm, double arbFeedforward, int slot);

    /**
     * Run the motor in velocity mode, in RPM.
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
     * Run the motor in position mode, in motor rotations.
     * 
     * @param rotations
     *            Rotations to run.
     * @param arbFeedforward
     *            Arbitrary feed-forward to pass to the motor controller, in volts.
     * @param slot
     *            PID slot to run.
     */
    public void setPositionMotorRotations(double rotations, double arbFeedforward, int slot);

    /**
     * Run the motor in position mode, in motor rotations.
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
     * Sets the encoder's position, in motor rotations.
     * 
     * @param rotations
     *            Rotations to set the encoder to.
     */
    public void setEncoderPositionMotorRotations(double rotations);

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
     * Runs the motor in motion magic mode, in motor rotations.
     * </p>
     * Not currently supported by SparkMAX.
     * 
     * @param rotations
     *            Rotations to run.
     * @param arbFeedforward
     *            Arbitrary feed-forward to pass to the motor controller, in volts.
     * @param slot
     *            PID slot to run.
     */
    public void setMotionMagicMotorRotations(double rotations, double arbFeedforward, int slot);

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
     * Get the motor velocity, in RPM.
     * 
     * @return Velocity in RPM combined with the timestamp of the received data.
     */
    public DataSignal<Double> getVelocityRPM();

    /**
     * Get the motor velocity, in NU.
     * </p>
     * NU/100ms for Talons, RPM for SparkMAX.
     * 
     * @return Velocity in NU combined with the timestamp of the received data.
     */
    public DataSignal<Double> getVelocityNU();

    /**
     * Get the motor position, in motor rotations.
     * 
     * @return Position in motor rotations.
     */
    public DataSignal<Double> getPositionMotorRotations();

    /**
     * Get the motor position, in NU.
     * 2048 NU per rotation for TalonFX, 4096 for TalonSRX, and usually 1 for
     * SparkMAX.
     * 
     * @return Position in NU combined with the timestamp of the received data.
     */
    public DataSignal<Double> getPositionNU();

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
     * @deprecated Not implemented for v6 TalonFX due to improved units.
     *             </p>
     *             Calculate the desired feed-forward, given a percent output and
     *             the NU that
     *             the PID controller should return.
     *             </p>
     *             Example process: Run the motor at 100% output (safely).
     *             </p>
     *             Get the motor's velocity in NU (i.e. 22000 NU/100ms for a
     *             free-spinning
     *             Falcon, 11000 RPM for a free-spinning NEO 550)
     *             </p>
     *             Pass 1 to percentOutput, and your recorded velocity to
     *             desiredOutputNU.
     * 
     * @param percentOutput
     *            Percent output of the motor (0-1).
     * @param desiredOutputNU
     *            Velocity in NU.
     * 
     * @return A calculated feed forward.
     *         </p>
     *         For Talons, this will be in the 0.005-0.2 range.
     *         </p>
     *         For SparkMAXes, this will be a very small number.
     */
    @Deprecated(forRemoval = true)
    public double calculateFeedForward(double percentOutput, double desiredOutputNU);

    /**
     * @deprecated This method is being replaced with a new conversion API.
     *             </p>
     *             Get the counts per revolution for the encoder when in velocity
     *             mode.
     */
    @Deprecated(forRemoval = true)
    public double getVelocityEncoderCPR();

    /**
     * @deprecated This method is being replaced with a new conversion API.
     *             </p>
     *             Get the counts per revolution for the encoder when in velocity
     *             mode.
     */
    @Deprecated(forRemoval = true)
    public double getPositionEncoderCPR();

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

    /**
     * @deprecated This method is being replaced with a new conversion API.
     * </p>
     * Set the encoder "distance per pulse". This can essentially be described as
     * the circumference of the wheel divided by the
     * CPR of the encoder. For example, with a 4 inch (.1 meter) wheel, on a 1:1
     * TalonFX:
     * 
     * <pre>
     * <code>
     * talon.setDistancePerPulse((.1 * Math.PI) / 2048);
     * </code>
     * </pre>
     * 
     * @param dpr
     *            The calculated Distance per Pulse.
     */
    @Deprecated(forRemoval = true)
    public void setDistancePerPulse(double dpr);

    @Deprecated(forRemoval = true)
    public double getDistancePerPulse();

    /**
     * @deprecated This method is being replaced with a new conversion API.
     * </p>
     * Set the encoder distance per pulse to meters per second.
     * 
     * @param wheelDiameter
     *            The diameter of the driven wheel, in inches.
     * @param encoderGearRatio
     *            The gear ratio between the encoder and the wheel
     *            (1 if the encoder is mounted directly on the wheel)
     */
    @Deprecated(forRemoval = true)
    default void setDistancePerPulse(Distance wheelDiameter, double encoderGearRatio) {
        setDistancePerPulse((wheelDiameter.getAsMeters() * Math.PI) / encoderGearRatio);
    }

    /**
     * @deprecated This method is being replaced with a new conversion API.
     * </p>
     * Get the traveled distance of the encoder, scaled from the distance per pulse.
     * 
     * @return Traveled motor distance, in whatever units were passed in
     *         setDistancePerPulse, combined with the timestamp of the received
     *         data.
     */
    @Deprecated(forRemoval = true)
    default DataSignal<Double> getDistance() {
        DataSignal<Double> position = getPositionNU();
        position.Value *= (getDistancePerPulse() / getPositionEncoderCPR()) / 10.;
        return position;
    }

    /**
     * @deprecated This method is being replaced with a new conversion API.
     * </p>
     * Get the current velocity of the encoder, scaled from the distance per pulse.
     * 
     * @return Current motor velocity, in whatever units were passed in
     *         setDistancePerPulse, combined with the timestamp of the received
     *         data.
     */
    @Deprecated(forRemoval = true)
    default DataSignal<Double> getRate() {
        DataSignal<Double> velocity = getVelocityNU();
        velocity.Value *= (getDistancePerPulse() / getVelocityEncoderCPR());
        return velocity;
    }

    /**
     * @deprecated This method is being replaced with a new conversion API.
     * </p>
     * Run the motor at the specified speed, scaled from the distance per pulse.
     * 
     * @param velocity
     *            Target motor velocity, in whatever units were passed in
     *            setDistancePerPulse
     */
    @Deprecated(forRemoval = true)
    default void setRate(double velocity) {
        setRate(velocity, 0, 0);
    }

    /**
     * @deprecated This method is being replaced with a new conversion API.
     * </p>
     * Run the motor at the specified speed, scaled from the distance per pulse.
     * 
     * @param velocity
     *            Target motor velocity, in whatever units were passed in
     *            setDistancePerPulse
     * @param arbFeedforward
     *            Arbitrary feed forward to pass to the motor controller, in volts.
     */
    @Deprecated(forRemoval = true)
    default void setRate(double velocity, double arbFeedforward) {
        setRate(velocity, arbFeedforward, 0);
    }

    /**
     * @deprecated This method is being replaced with a new conversion API.
     * </p>
     * Run the motor at the specified speed, scaled from the distance per pulse.
     * 
     * @param velocity
     *            Target motor velocity, in whatever units were passed in
     *            setDistancePerPulse
     * @param arbFeedforward
     *            Arbitrary feed forward to pass to the motor controller, in volts.
     * @param slot
     *            The PID slot to use.
     */
    @Deprecated(forRemoval = true)
    default void setRate(double velocity, double arbFeedforward, int slot) {
        setVelocityNU(velocity / (getDistancePerPulse() / getVelocityEncoderCPR()), arbFeedforward, slot);
    }
}
