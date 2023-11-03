// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.motor;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import frc.lib.beaklib.pid.BeakPIDConstants;

/** Common motor controller interface for TalonFX/Falcon 500. */
public class BeakTalonFX extends WPI_TalonFX implements BeakMotorController {
    private double m_velocityConversionConstant = 2048. / 600.;
    private double m_positionConversionConstant = 2048.;
    private double m_gearRatio = 1.;
    private Measure<Distance> m_wheelDiameter = Inches.of(4.);

    private double m_arbFeedforward = 0.;
    private int m_slot = 0;

    public BeakTalonFX(int port, String canBus) {
        super(port, canBus);
        super.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        super.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    }

    public BeakTalonFX(int port) {
        this(port, "");
    }

    @Override
    public void setSlot(int slot) {
        m_slot = slot;
        super.selectProfileSlot(slot, 0);
    }

    @Override
    public void setBrake(boolean brake) {
        super.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    public void setVelocityNU(double nu) {
        super.set(ControlMode.Velocity, nu, DemandType.ArbitraryFeedForward, m_arbFeedforward / 12.);
    }

    @Override
    public void setPositionNU(double nu) {
        super.set(ControlMode.Position, nu, DemandType.ArbitraryFeedForward, m_arbFeedforward / 12.);
    }

    @Override
    public void setEncoderPositionNU(double nu) {
        super.setSelectedSensorPosition(nu);
    }

    @Override
    public void setMotionMagicNU(double nu) {
        super.set(ControlMode.MotionMagic, nu, DemandType.ArbitraryFeedForward, m_arbFeedforward / 12.);
    }

    @Override
    public DataSignal<Double> getVelocityNU() {
        return new DataSignal<Double>(super.getSelectedSensorVelocity());
    }

    @Override
    public DataSignal<Double> getPositionNU(boolean latencyCompensated) {
        return new DataSignal<Double>(super.getSelectedSensorPosition());
    }

    @Override
    public DataSignal<Double> getOutputVoltage() {
        return new DataSignal<Double>(super.getMotorOutputVoltage());
    }

    public BeakPIDConstants getPID() {
        SlotConfiguration config = new SlotConfiguration();
        super.getSlotConfigs(config, m_slot, 50);
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
        super.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, amps, amps + 5, 0.1));
    }

    @Override
    public void restoreFactoryDefault() {
        super.configFactoryDefault();
    }

    @Override
    public void setAllowedClosedLoopError(double error) {
        super.configAllowableClosedloopError(m_slot, error);
    }

    @Override
    public void setVoltageCompensationSaturation(double saturation) {
        super.enableVoltageCompensation(saturation > 0.);
        super.configVoltageCompSaturation(saturation);
    }

    @Override
    public void setMotionMagicAcceleration(double accel) {
        super.configMotionAcceleration(accel);
    }

    @Override
    public void setMotionMagicCruiseVelocity(double velocity) {
        super.configMotionCruiseVelocity(velocity);
    }

    @Override
    public void set(double percentOutput) {
        super.set(ControlMode.PercentOutput, percentOutput, DemandType.ArbitraryFeedForward, m_arbFeedforward);
    }

    @Override
    public DataSignal<Double> getSuppliedVoltage() {
        return new DataSignal<Double>(getBusVoltage());
    }

    @Override
    public void setPID(BeakPIDConstants constants) {
        super.config_kP(m_slot, constants.kP);
        super.config_kI(m_slot, constants.kI);
        super.config_kD(m_slot, constants.kD);
        super.config_kF(m_slot, constants.kF);
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
    public void setWheelDiameter(Measure<Distance> diameter) {
        m_wheelDiameter = diameter;
    }

    @Override
    public Measure<Distance> getWheelDiameter() {
        return m_wheelDiameter;
    }

    @Override
    public void setNextArbFeedforward(double arbFeedforward) {
        m_arbFeedforward = arbFeedforward;
    }
}
