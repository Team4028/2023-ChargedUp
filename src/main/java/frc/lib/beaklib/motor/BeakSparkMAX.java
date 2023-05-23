// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import frc.lib.beaklib.pid.BeakPIDConstants;

/** Common motor controller interface for REV Spark MAX. */
public class BeakSparkMAX extends CANSparkMax implements BeakMotorController {
    private RelativeEncoder m_encoder;
    private SparkMaxPIDController m_pid;

    private SparkMaxLimitSwitch m_revLimitSwitch;
    private SparkMaxLimitSwitch m_fwdLimitSwitch;

    private double m_distancePerPulse;

    public BeakSparkMAX(int port) {
        super(port, MotorType.kBrushless);

        resetControllers();
    }

    @Override
    public void setBrake(boolean brake) {
        super.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setVelocityRPM(double rpm, double arbFeedforward, int slot) {
        setVelocityNU(rpm, arbFeedforward, slot);
    }

    @Override
    public void setVelocityNU(double nu, double arbFeedforward, int slot) {
        m_pid.setReference(nu, ControlType.kVelocity, slot, arbFeedforward);
    }

    @Override
    public void setPositionMotorRotations(double rotations, double arbFeedforward, int slot) {
        setPositionNU(rotations * m_encoder.getCountsPerRevolution(), arbFeedforward, slot);
    }

    @Override
    public void setPositionNU(double nu, double arbFeedforward, int slot) {
        m_pid.setReference(nu, ControlType.kPosition, slot, arbFeedforward);
    }

    @Override
    public void setEncoderPositionMotorRotations(double rotations) {
        setEncoderPositionNU(rotations * m_encoder.getCountsPerRevolution());
    }

    @Override
    public void setEncoderPositionNU(double nu) {
        m_encoder.setPosition(nu);
    }

    @Override
    public void setMotionMagicMotorRotations(double rotations, double arbFeedforward, int slot) {
        setMotionMagicNU(rotations, arbFeedforward, slot);
    }

    @Override
    public void setMotionMagicNU(double nu, double arbFeedforward, int slot) {
        m_pid.setReference(nu, ControlType.kSmartMotion, slot, arbFeedforward);
    }

    @Override
    public DataSignal<Double> getVelocityRPM() {
        return getVelocityNU();
    }

    @Override
    public DataSignal<Double> getVelocityNU() {
        return new DataSignal<Double>(m_encoder.getVelocity());
    }

    @Override
    public DataSignal<Double> getPositionMotorRotations() {
        return new DataSignal<Double>(getPositionNU().Value / m_encoder.getCountsPerRevolution());
    }

    @Override
    public DataSignal<Double> getPositionNU() {
        return new DataSignal<Double>(m_encoder.getPosition());
    }

    @Override
    public double calculateFeedForward(double percentOutput, double desiredOutputNU) {
        return percentOutput / desiredOutputNU;
    }

    @Override
    public double getVelocityEncoderCPR() {
        // return encoder.getCountsPerRevolution();
        return 600.; // TEMP
    }

    @Override
    public double getPositionEncoderCPR() {
        // return encoder.getCountsPerRevolution();
        return 1.; // TEMP
    }

    @Override
    public void setReverseLimitSwitchNormallyClosed(boolean normallyClosed) {
        m_revLimitSwitch = super.getReverseLimitSwitch(normallyClosed ? Type.kNormallyClosed : Type.kNormallyOpen);
    }

    @Override
    public void setForwardLimitSwitchNormallyClosed(boolean normallyClosed) {
        m_fwdLimitSwitch = super.getForwardLimitSwitch(normallyClosed ? Type.kNormallyClosed : Type.kNormallyOpen);
    }

    @Override
    public DataSignal<Boolean> getReverseLimitSwitch() {
        return new DataSignal<Boolean>(m_revLimitSwitch.isPressed());
    }

    @Override
    public DataSignal<Boolean> getForwardLimitSwitch() {
        return new DataSignal<Boolean>(m_fwdLimitSwitch.isPressed());
    }

    @Override
    public void setSupplyCurrentLimit(int amps) {
        super.setSmartCurrentLimit(amps);
    }

    @Override
    public void setStatorCurrentLimit(int amps) {
        System.err.println("REV Spark MAX does not support stator current limiting.");
    }

    @Override
    public void restoreFactoryDefault() {
        super.restoreFactoryDefaults();
    }

    @Override
    public void setAllowedClosedLoopError(double error, int slot) {
        m_pid.setSmartMotionAllowedClosedLoopError(error, slot);
    }

    @Override
    public void setVoltageCompensationSaturation(double saturation) {
        super.enableVoltageCompensation(saturation);
    }

    @Override
    public void setMotionMagicAcceleration(double accel, int slot) {
        m_pid.setSmartMotionMaxAccel(accel, slot);
    }

    @Override
    public void setMotionMagicCruiseVelocity(double velocity, int slot) {
        m_pid.setSmartMotionMaxVelocity(velocity, slot);
    }

    @Override
    public void set(double percentOutput, double arbFeedforward) {
        m_pid.setReference(percentOutput, ControlType.kDutyCycle, 0, arbFeedforward);
    }

    @Override
    public void setStatusPeriod(int value, int period) {
        super.setPeriodicFramePeriod(PeriodicFrame.fromId(value), period);
    }

    @Override
    public void setDistancePerPulse(double dpr) {
        m_distancePerPulse = dpr;
    }

    @Override
    public double getDistancePerPulse() {
        return m_distancePerPulse;
    }

    @Override
    public void setPID(BeakPIDConstants constants, int slot) {
        m_pid.setP(constants.kP, slot);
        m_pid.setI(constants.kI, slot);
        m_pid.setD(constants.kD, slot);
        m_pid.setFF(constants.kF, slot);
    }

    @Override
    public BeakPIDConstants getPID(int slot) {
        return new BeakPIDConstants(
            m_pid.getP(slot),
            m_pid.getI(slot),
            m_pid.getD(slot),
            m_pid.getFF(slot)
        );
    }

    @Override
    public DataSignal<Double> getSuppliedVoltage() {
        return new DataSignal<Double>(getBusVoltage());
    }

    private void resetControllers() {
        m_encoder = super.getEncoder();
        m_pid = super.getPIDController();

        m_revLimitSwitch = super.getReverseLimitSwitch(Type.kNormallyOpen);
        m_fwdLimitSwitch = super.getForwardLimitSwitch(Type.kNormallyOpen);
    }
}
