// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.motor;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.utilities.units.Distance;

/** Common interface for all motor controllers. */
public interface BeakMotorController extends MotorController {
    /**
     * Set PIDF gains.
     * 
     * @param p    Proportional gain.
     * @param i    Integral gain.
     * @param d    Derivative gain.
     * @param f    Feed-Forward gain.
     * @param slot The slot to set these values in.
     */
    default void setPIDF(
            double p,
            double i,
            double d,
            double f,
            int slot) {
        setP(p, slot);
        setI(i, slot);
        setD(d, slot);
        setF(f, slot);
    }

    /**
     * Set the motor to be on brake or coast mode.
     * 
     * @param brake True = brake, False = coast
     */
    public void setBrake(boolean brake);

    /**
     * Run the motor in velocity mode, in RPM.
     * 
     * @param rpm            RPM to run.
     * @param arbFeedforward Arbitrary feed-forward to pass to the motor controller, in volts.
     */
    public void setVelocityRPM(double rpm, double arbFeedforward, int slot);

    /**
     * Run the motor in velocity mode, in RPM.
     * 
     * @param rpm RPM to run.
     */
    default void setVelocityRPM(double rpm, int slot) {
        setVelocityRPM(rpm, 0, slot);
    }

    /**
     * Run the motor in velocity mode, in NU.
     * </p>
     * NU/100ms for Talons, RPM for SparkMAX.
     * 
     * @param nu             NU to run.
     * @param arbFeedforward Arbitrary feed-forward to pass to the motor controller, in volts.
     * @param slot           PID slot to run.
     */
    public void setVelocityNU(double nu, double arbFeedforward, int slot);

    /**
     * Run the motor in velocity mode, in NU.
     * </p>
     * NU/100ms for Talons, RPM for SparkMAX.
     * 
     * @param nu   NU to run.
     * @param slot PID slot to run.
     */
    default void setVelocityNU(double nu, int slot) {
        setVelocityNU(nu, 0, slot);
    }

    /**
     * Run the motor in velocity mode, in NU.
     * </p>
     * NU/100ms for Talons, RPM for SparkMAX.
     * 
     * @param nu NU to run.
     */
    default void setVelocityNU(double nu) {
        setVelocityNU(nu, 0, 0);
    }

    /**
     * Run the motor in position mode, in motor rotations.
     * 
     * @param rotations      Rotations to run.
     * @param arbFeedforward Arbitrary feed-forward to pass to the motor controller, in volts.
     * @param slot           PID slot to run.
     */
    public void setPositionMotorRotations(double rotations, double arbFeedforward, int slot);

    /**
     * Run the motor in position mode, in motor rotations.
     * 
     * @param rotations Rotations to run.
     * @param slot      PID slot to run.
     */
    default void setPositionMotorRotations(double rotations, int slot) {
        setPositionMotorRotations(rotations, 0, slot);
    }

    /**
     * Run the motor in position mode, in motor rotations.
     * 
     * @param rotations Rotations to run.
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
     * @param nu             NU to run.
     * @param arbFeedforward Arbitrary feed-forward to pass to the motor controller, in volts.
     * @param slot           PID slot to run.
     */
    public void setPositionNU(double nu, double arbFeedforward, int slot);

    /**
     * Run the motor in position mode, in NU.
     * </p>
     * 2048 NU per rotation for TalonFX, 4096 for TalonSRX, and usually 1 for
     * SparkMAX.
     * 
     * @param nu   NU to run.
     * @param slot PID slot to run.
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
     * @param nu NU to run.
     */
    default void setPositionNU(double nu) {
        setPositionNU(nu, 0, 0);
    }

    /**
     * Sets the encoder's position, in motor rotations.
     * 
     * @param rotations Rotations to set the encoder to.
     */
    public void setEncoderPositionMotorRotations(double rotations);

    /**
     * Sets the encoder's position, in NU.
     * </p>
     * 2048 NU per rotation for TalonFX, 4096 for TalonSRX, and usually 1 for
     * SparkMAX.
     * 
     * @param nu NU to set the encoder to.
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
     * @param rotations      Rotations to run.
     * @param arbFeedforward Arbitrary feed-forward to pass to the motor controller, in volts.
     * @param slot           PID slot to run.
     */
    public void setMotionMagicMotorRotations(double rotations, double arbFeedforward, int slot);

    /**
     * Runs the motor in motion magic mode, in motor rotations.
     * </p>
     * Not currently supported by SparkMAX.
     * 
     * @param rotations Rotations to run.
     * @param slot      PID slot to run.
     */
    default void setMotionMagicMotorRotations(double rotations, int slot) {
        setMotionMagicMotorRotations(rotations, 0, slot);
    }

    /**
     * Runs the motor in motion magic mode, in motor rotations.
     * </p>
     * Not currently supported by SparkMAX.
     * 
     * @param rotations Rotations to run.
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
     * @param nu             NU to run.
     * @param arbFeedforward Arbitrary feed-forward to pass to the motor controller, in volts.
     * @param slot           PID slot to run.
     */
    public void setMotionMagicNU(double nu, double arbFeedforward, int slot);

    /**
     * Runs the motor in motion magic mode, in NU.
     * </p>
     * 2048 NU per rotation for TalonFX, 4096 for TalonSRX, and usually 1 for
     * SparkMAX.
     * 
     * @param nu   NU to run.
     * @param slot PID slot to run.
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
     * @param nu NU to run.
     */
    default void setMotionMagicNU(double nu) {
        setMotionMagicNU(nu, 0, 0);
    }

    /**
     * Get the motor velocity, in RPM.
     * 
     * @return Velocity in RPM.
     */
    public double getVelocityRPM();

    /**
     * Get the motor velocity, in NU.
     * </p>
     * NU/100ms for Talons, RPM for SparkMAX.
     * 
     * @return Velocity in NU.
     */
    public double getVelocityNU();

    /**
     * Get the motor position, in motor rotations.
     * 
     * @return Position in motor rotations.
     */
    public double getPositionMotorRotations();

    /**
     * Get the motor position, in NU.
     * 2048 NU per rotation for TalonFX, 4096 for TalonSRX, and usually 1 for
     * SparkMAX.
     * 
     * @return Position in NU.
     */
    public double getPositionNU();

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
     * Get the voltage currently being run to the motor controller.
     */
    public double getBusVoltage();

    /**
     * Get the current applied voltage to the motor controller.
     * 
     * @return Applied voltage.
     */
    default double getOutputVoltage() {
        return get() * getBusVoltage();
    }

    /**
     * Get the P value of the PID controller.
     * 
     * @param slot Slot to get from.
     * @return Proportional gain.
     */
    public double getP(int slot);

    /**
     * Get the I value of the PID controller.
     * 
     * @param slot Slot to get from.
     * @return Integral gain.
     */
    public double getI(int slot);

    /**
     * Get the D value of the PID controller.
     * 
     * @param slot Slot to get from.
     * @return Derivative gain.
     */
    public double getD(int slot);

    /**
     * Get the F value of the PID controller.
     * 
     * @param slot Slot to get from.
     * @return Feed-Forward gain.
     */
    public double getF(int slot);

    /**
     * Set the P value of the PID controller.
     * 
     * @param p    Proportional gain.
     * @param slot Slot to set to.
     */
    public void setP(double p, int slot);

    /**
     * Set the I value of the PID controller.
     * 
     * @param i    Integral gain.
     * @param slot Slot to set to.
     */
    public void setI(double i, int slot);

    /**
     * Set the D value of the PID controller.
     * 
     * @param D    Derivative gain.
     * @param slot Slot to set to.
     */
    public void setD(double d, int slot);

    /**
     * Set the F value of the PID controller.
     * 
     * @param f    Feed-Forward gain.
     * @param slot Slot to set to.
     */
    public void setF(double f, int slot);

    /**
     * Calculate the desired feed-forward, given a percent output and the NU that
     * the PID controller should return.
     * </p>
     * Example process: Run the motor at 100% output (safely).
     * </p>
     * Get the motor's velocity in NU (i.e. 22000 NU/100ms for a free-spinning
     * Falcon, 11000 RPM for a free-spinning NEO 550)
     * </p>
     * Pass 1 to percentOutput, and your recorded velocity to desiredOutputNU.
     * 
     * @param percentOutput   Percent output of the motor (0-1).
     * @param desiredOutputNU Velocity in NU.
     * 
     * @return A calculated feed forward.
     *         </p>
     *         For Talons, this will be in the 0.005-0.2 range.
     *         </p>
     *         For SparkMAXes, this will be a very small number.
     */
    public double calculateFeedForward(double percentOutput, double desiredOutputNU);

    /**
     * Get the counts per revolution for the encoder when in velocity mode.
     */
    public double getVelocityEncoderCPR();

    /**
     * Get the counts per revolution for the encoder when in velocity mode.
     */
    public double getPositionEncoderCPR();

    /**
     * Set the reverse limit switch's default state
     * 
     * @param normallyClosed True if its normal state is "closed", false if its
     *                       normal state is "open"
     */
    public void setReverseLimitSwitchNormallyClosed(boolean normallyClosed);

    /**
     * Set the forward limit switch's default state
     * 
     * @param normallyClosed True if its normal state is "closed", false if its
     *                       normal state is "open"
     */
    public void setForwardLimitSwitchNormallyClosed(boolean normallyClosed);

    /**
     * Whether or not the limit switch is closed. This is independent of the
     * polarity (normally-closed) option on
     * CTRE devices, but on Spark MAXes, it is dependent--i.e. returning true if the
     * limit switch is not pressed,
     * when it's configured to be normally closed.
     */
    public boolean getReverseLimitSwitch();

    /**
     * Whether or not the limit switch is closed. This is independent of the
     * polarity (normally-closed) option on
     * CTRE devices, but on Spark MAXes, it is dependent--i.e. returning true if the
     * limit switch is not pressed,
     * when it's configured to be normally closed.
     */
    public boolean getForwardLimitSwitch();

    /**
     * Set the supply (PDH to controller) current limit.
     * </p>
     * 
     * For Talons, the "tripping" point is set to this plus 5, and the time to trip
     * back to the limit is set to 0.1 seconds.
     * 
     * @param amps The maximum amps to allow the motor controller to receive.
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
     * @param amps The maximum amps to allow the motor controller to send.
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
     * @param error Error deadband.
     * @param slot  Slot to set to.
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
     * @param saturation Saturation.
     */
    public void setVoltageCompensationSaturation(double saturation);

    /**
     * Set the Motion Magic cruise velocity.
     * </p>
     * 
     * See CTRE's Motion Magic documentation, or REV's Smart Motion example
     * to see what this means.
     * 
     * @param velocity Cruise velocity, in NU.
     */
    public void setMotionMagicCruiseVelocity(double velocity, int slot);

    /**
     * Set the Motion Magic acceleration.
     * </p>
     * 
     * See CTRE's Motion Magic documentation, or REV's Smart Motion example
     * to see what this means.
     * 
     * @param accel Acceleration, in NU per second.
     */
    public void setMotionMagicAcceleration(double accel, int slot);

    /**
     * Set the motor controller's speed, in range [-1.0, 1.0], with an arbitrary
     * feed forward.
     * 
     * @param percentOutput  Percent output to pass to the motor controller.
     * @param arbFeedforward Arbitrary feed-forward to pass to the motor controller, in volts.
     */
    public void set(double percentOutput, double arbFeedforward);

    /**
     * Set a status frame period of the motor controller.</p>
     * 
     * Values are dependent upon the individual motor controller.
     * Pass a corresponding value for your motor controller (by appending .value to the enum), i.e.:</p>
     * 
     * <pre>
     * <code>
     * controller.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General.value, 11); // TalonFX
     * sparkController.setStatusFramePeriod(PeriodicFrame.kStatus0.value, 1); // SparkMAX
     * </code>
     * </pre>
     * 
     * @param value Value of the status frame, from an enum.
     * @param period Period of the status frame, in ms.
     */
    public void setStatusPeriod(int value, int period);

    /**
     * Set the encoder "distance per pulse". This can essentially be described as the circumference of the wheel divided by the 
     * CPR of the encoder. For example, with a 4 inch (.1 meter) wheel, on a 1:1 TalonFX:
     * 
     * <pre>
     * <code>
     * talon.setDistancePerPulse((.1 * Math.PI) / 2048);
     * </code>
     * </pre>
     * 
     * @param dpr The calculated Distance per Pulse.
     */
    public void setDistancePerPulse(double dpr);

    public double getDistancePerPulse();

    /**
     * Set the encoder distance per pulse to meters per second.
     * 
     * @param wheelDiameter The diameter of the driven wheel, in inches.
     * @param encoderGearRatio The gear ratio between the encoder and the wheel
     * (1 if the encoder is mounted directly on the wheel)
     */
    default void setDistancePerPulse(Distance wheelDiameter, double encoderGearRatio) {
        setDistancePerPulse((wheelDiameter.getAsMeters() * Math.PI) / encoderGearRatio);
    }

    /**
     * Get the traveled distance of the encoder, scaled from the distance per pulse.
     * @return Traveled motor distance, in whatever units were passed in setDistancePerPulse
     */
    default double getDistance() {
        return getPositionNU() * (getDistancePerPulse() / getPositionEncoderCPR()) / 10.;
    }

    /**
     * Get the current velocity of the encoder, scaled from the distance per pulse.
     * @return Current motor velocity, in whatever units were passed in setDistancePerPulse
     */
    default double getRate() {
        return getVelocityNU() * (getDistancePerPulse() / getVelocityEncoderCPR());
    }

    /**
     * Run the motor at the specified speed, scaled from the distance per pulse.
     * @param velocity Target motor velocity, in whatever units were passed in setDistancePerPulse
     */
    default void setRate(double velocity) {
        setRate(velocity, 0, 0);
    }

    /**
     * Run the motor at the specified speed, scaled from the distance per pulse.
     * @param velocity Target motor velocity, in whatever units were passed in setDistancePerPulse
     * @param arbFeedforward Arbitrary feed forward to pass to the motor controller, in volts.
     */
    default void setRate(double velocity, double arbFeedforward) {
        setRate(velocity, arbFeedforward, 0);
    }

    /**
     * Run the motor at the specified speed, scaled from the distance per pulse.
     * @param velocity Target motor velocity, in whatever units were passed in setDistancePerPulse
     * @param arbFeedforward Arbitrary feed forward to pass to the motor controller, in volts.
     * @param slot The PID slot to use.
     */
    default void setRate(double velocity, double arbFeedforward, int slot) {
        setVelocityNU(velocity / (getDistancePerPulse() / getVelocityEncoderCPR()), arbFeedforward, slot);
    }
}
