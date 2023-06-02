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
import frc.lib.beaklib.units.Distance;

/** Common motor controller interface for REV Spark MAX. */
public class BeakSparkMAX extends CANSparkMax implements BeakMotorController {
    private RelativeEncoder m_encoder;
    private SparkMaxPIDController m_pid;

    private SparkMaxLimitSwitch m_revLimitSwitch;
    private SparkMaxLimitSwitch m_fwdLimitSwitch;

    private double m_velocityConversionConstant = 1.;
    private double m_positionConversionConstant = 1.;
    private double m_gearRatio = 1.;
    private Distance m_wheelDiameter = Distance.fromInches(4.);

    public BeakSparkMAX(int port) {
        super(port, MotorType.kBrushless);

        resetControllers();
    }

    @Override
    public void setBrake(boolean brake) {
        super.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setVelocityNU(double nu, double arbFeedforward, int slot) {
        m_pid.setReference(nu, ControlType.kVelocity, slot, arbFeedforward);
    }

    @Override
    public void setPositionNU(double nu, double arbFeedforward, int slot) {
        m_pid.setReference(nu, ControlType.kPosition, slot, arbFeedforward);
    }

    @Override
    public void setEncoderPositionNU(double nu) {
        m_encoder.setPosition(nu);
    }

    @Override
    public void setMotionMagicNU(double nu, double arbFeedforward, int slot) {
        m_pid.setReference(nu, ControlType.kSmartMotion, slot, arbFeedforward);
    }

    @Override
    public DataSignal<Double> getVelocityNU() {
        return new DataSignal<Double>(m_encoder.getVelocity());
    }

    @Override
    public DataSignal<Double> getPositionNU(boolean latencyCompensated) {
        return new DataSignal<Double>(m_encoder.getPosition());
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
            m_pid.getFF(slot));
    }

    @Override
    public DataSignal<Double> getSuppliedVoltage() {
        return new DataSignal<Double>(getBusVoltage());
    }

    @Override
    public void setVelocityConversionConstant(double constant) {
        m_velocityConversionConstant = constant;
    }

    @Override
    public double getVelocityConversionConstant() {
        return m_velocityConversionConstant;
    }

    @Override
    public void setPositionConversionConstant(double constant) {
        m_positionConversionConstant = constant;
    }

    @Override
    public double getPositionConversionConstant() {
        return m_positionConversionConstant;
    }

    @Override
    public void setEncoderGearRatio(double ratio) {
        m_gearRatio = ratio;
    }

    @Override
    public double getEncoderGearRatio() {
        return m_gearRatio;
    }

    @Override
    public void setWheelDiameter(Distance diameter) {
        m_wheelDiameter = diameter;
    }

    @Override
    public Distance getWheelDiameter() {
        return m_wheelDiameter;
    }

    private void resetControllers() {
        m_encoder = super.getEncoder();
        m_pid = super.getPIDController();

        m_revLimitSwitch = super.getReverseLimitSwitch(Type.kNormallyOpen);
        m_fwdLimitSwitch = super.getForwardLimitSwitch(Type.kNormallyOpen);
    }
}
