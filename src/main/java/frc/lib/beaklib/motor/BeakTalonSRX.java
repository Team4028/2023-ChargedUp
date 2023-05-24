// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.motor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.lib.beaklib.pid.BeakPIDConstants;
import frc.lib.beaklib.units.Distance;

/** Common motor controller interface for Talon SRX. */
public class BeakTalonSRX extends WPI_TalonSRX implements BeakMotorController {
    private double m_velocityConversionConstant = 600. / 4096.;
    private double m_positionConversionConstant = 4096.;
    private double m_gearRatio = 1.;
    private Distance m_wheelDiameter = Distance.fromInches(4.);

    public BeakTalonSRX(int port) {
        super(port);
    }

    public void setSlot(int slot) {
        super.selectProfileSlot(slot, 0);
    }

    @Override
    public void setBrake(boolean brake) {
        super.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    public void setVelocityRPM(double rpm, double arbFeedforward, int slot) {
        setVelocityNU(rpm * 4096 / 600, arbFeedforward, slot);
    }

    @Override
    public void setVelocityNU(double nu, double arbFeedforward, int slot) {
        setSlot(slot);
        super.set(ControlMode.Velocity, nu, DemandType.ArbitraryFeedForward, arbFeedforward / 12.);
    }

    @Override
    public void setPositionMotorRotations(double rotations, double arbFeedforward, int slot) {
        setPositionNU(rotations * 4096, arbFeedforward, slot);
    }

    @Override
    public void setPositionNU(double nu, double arbFeedforward, int slot) {
        setSlot(slot);
        super.set(ControlMode.Position, nu, DemandType.ArbitraryFeedForward, arbFeedforward / 12.);
    }

    @Override
    public void setEncoderPositionMotorRotations(double rotations) {
        setEncoderPositionNU(rotations * 4096);
    }

    @Override
    public void setEncoderPositionNU(double nu) {
        super.setSelectedSensorPosition(nu);
    }

    @Override
    public void setMotionMagicMotorRotations(double rotations, double arbFeedforward, int slot) {
        setMotionMagicNU(rotations * 4096, arbFeedforward, slot);
    }

    @Override
    public void setMotionMagicNU(double nu, double arbFeedforward, int slot) {
        setSlot(slot);
        super.set(ControlMode.MotionMagic, nu, DemandType.ArbitraryFeedForward, arbFeedforward / 12.);
    }

    @Override
    public DataSignal<Double> getVelocityRPM() {
        return new DataSignal<Double>(getVelocityNU().Value * 4096 / 600);
    }

    @Override
    public DataSignal<Double> getVelocityNU() {
        return new DataSignal<Double>(super.getSelectedSensorVelocity());
    }

    @Override
    public DataSignal<Double> getPositionMotorRotations() {
        return new DataSignal<Double>(getPositionNU().Value * 4096);
    }

    @Override
    public DataSignal<Double> getPositionNU() {
        return new DataSignal<Double>(super.getSelectedSensorPosition());
    }

    @Override
    public DataSignal<Double> getOutputVoltage() {
        return new DataSignal<Double>(super.getMotorOutputVoltage());
    }

    @Override
    public BeakPIDConstants getPID(int slot) {
        SlotConfiguration config = new SlotConfiguration();
        super.getSlotConfigs(config, slot, 50);
        return new BeakPIDConstants(config);
    }

    public TalonSRXSimCollection getTalonSRXSimCollection() {
        return super.getTalonSRXSimCollection();
    }

    public TalonFXSimCollection getTalonFXSimCollection() {
        return super.getTalonFXSimCollection();
    }

    @Override
    public void setReverseLimitSwitchNormallyClosed(boolean normallyClosed) {
        super.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
            normallyClosed ? LimitSwitchNormal.NormallyClosed : LimitSwitchNormal.NormallyOpen);
    }

    @Override
    public void setForwardLimitSwitchNormallyClosed(boolean normallyClosed) {
        super.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
            normallyClosed ? LimitSwitchNormal.NormallyClosed : LimitSwitchNormal.NormallyOpen);
    }

    @Override
    public DataSignal<Boolean> getReverseLimitSwitch() {
        return new DataSignal<Boolean>(super.isRevLimitSwitchClosed() == 1);
    }

    @Override
    public DataSignal<Boolean> getForwardLimitSwitch() {
        return new DataSignal<Boolean>(super.isFwdLimitSwitchClosed() == 1);
    }

    @Override
    public void setSupplyCurrentLimit(int amps) {
        super.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, amps, amps + 5, 0.1));
    }

    @Override
    public void setStatorCurrentLimit(int amps) {
        throw new RuntimeException("CTRE Talon SRX does not support Stator current limiting.");
    }

    @Override
    public void restoreFactoryDefault() {
        super.configFactoryDefault();
    }

    @Override
    public void setAllowedClosedLoopError(double error, int slot) {
        super.configAllowableClosedloopError(slot, error);
    }

    @Override
    public void setVoltageCompensationSaturation(double saturation) {
        super.enableVoltageCompensation(saturation > 0.);
        super.configVoltageCompSaturation(saturation);
    }

    @Override
    public void setMotionMagicAcceleration(double accel, int slot) {
        setSlot(slot);
        super.configMotionAcceleration(accel);
    }

    @Override
    public void setMotionMagicCruiseVelocity(double velocity, int slot) {
        setSlot(slot);
        super.configMotionCruiseVelocity(velocity);
    }

    @Override
    public void set(double percentOutput, double arbFeedforward) {
        super.set(ControlMode.PercentOutput, percentOutput, DemandType.ArbitraryFeedForward, arbFeedforward / 12.);
    }

    @Override
    public void setStatusPeriod(int value, int period) {
        super.setStatusFramePeriod(value, period);
    }

    @Override
    public void setPID(BeakPIDConstants constants, int slot) {
        super.config_kP(slot, constants.kP);
        super.config_kI(slot, constants.kI);
        super.config_kD(slot, constants.kD);
        super.config_kF(slot, constants.kF);
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
}
