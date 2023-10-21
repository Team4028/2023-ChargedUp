// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.motor;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.beaklib.pid.BeakPIDConstants;
import frc.lib.beaklib.units.Distance;

/** Add your docs here. */
public class BeakV6TalonFX extends TalonFX implements BeakMotorController {
    private TalonFXConfigurator m_configurator;
    private TalonFXConfiguration m_config = new TalonFXConfiguration();

    private DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0.);
    private VoltageOut m_voltageOut = new VoltageOut(0.);
    private VelocityDutyCycle m_velocityOut = new VelocityDutyCycle(0.);
    private VelocityVoltage m_velocityVoltage = new VelocityVoltage(0.);
    private PositionDutyCycle m_positionOut = new PositionDutyCycle(0.);
    private PositionVoltage m_positionVoltage = new PositionVoltage(0.);
    private MotionMagicDutyCycle m_motionMagicOut = new MotionMagicDutyCycle(0.);
    private MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(0.);

    private double m_velocityConversionConstant = 1. / 60.;
    private double m_positionConversionConstant = 1.;
    private double m_gearRatio = 1.;
    private Distance m_wheelDiameter = Distance.fromInches(4.);

    private boolean m_voltageCompEnabled = false;

    public BeakV6TalonFX(int port, String canBus) {
        super(port, canBus);
        m_configurator = super.getConfigurator();
        m_configurator.refresh(m_config);
    }

    public BeakV6TalonFX(int port) {
        this(port, "");
    }

    @Override
    public void setBrake(boolean brake) {
        // v6 is funky
        NeutralModeValue neutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        MotorOutputConfigs config = new MotorOutputConfigs();
        m_configurator.refresh(config);

        config.NeutralMode = neutralMode;

        m_configurator.apply(config, 0.1);
    }

    @Override
    public void setVelocityNU(double nu, double arbFeedforward, int slot) {
        if (m_voltageCompEnabled) {
            super.setControl(m_velocityVoltage
                .withFeedForward(arbFeedforward)
                .withSlot(slot)
                .withVelocity(nu));
        } else {
            super.setControl(m_velocityOut
                .withFeedForward(arbFeedforward)
                .withSlot(slot)
                .withVelocity(nu));
        }
    }

    @Override
    public void setPositionNU(double nu, double arbFeedforward, int slot) {
        if (m_voltageCompEnabled) {
            super.setControl(m_positionVoltage
                .withFeedForward(arbFeedforward)
                .withSlot(slot)
                .withPosition(nu));
        } else {
            super.setControl(m_positionOut
                .withFeedForward(arbFeedforward)
                .withSlot(slot)
                .withPosition(nu));
        }
    }

    @Override
    public void setEncoderPositionNU(double nu) {
        super.setPosition(nu, 0.1);
    }

    @Override
    public void setMotionMagicNU(double nu, double arbFeedforward, int slot) {
        if (m_voltageCompEnabled) {
            super.setControl(m_motionMagicVoltage
                .withFeedForward(arbFeedforward)
                .withSlot(slot)
                .withPosition(nu));
        } else {
            super.setControl(m_motionMagicOut
                .withFeedForward(arbFeedforward)
                .withSlot(slot)
                .withPosition(nu));
        }
    }

    @Override
    public DataSignal<Double> getVelocityNU() {
        return new DataSignal<Double>(super.getVelocity());
    }

    @Override
    public DataSignal<Double> getPositionNU(boolean latencyCompensated) {
        if (latencyCompensated) {
            return new DataSignal<Double>(
                StatusSignal.getLatencyCompensatedValue(super.getPosition(), super.getVelocity())
            );
        }
        return new DataSignal<Double>(super.getPosition());
    }

    @Override
    public DataSignal<Double> getSuppliedVoltage() {
        return new DataSignal<Double>(super.getSupplyVoltage());
    }

    @Override
    public void setPID(BeakPIDConstants constants, int slot) {
        // The v6 slot API is wacky
        switch (slot) {
            case 0:
                Slot0Configs slot0Config = new Slot0Configs();
                slot0Config.kP = constants.kP;
                slot0Config.kI = constants.kI;
                slot0Config.kD = constants.kD;
                slot0Config.kV = constants.kF;
                slot0Config.kS = constants.kS;
                m_configurator.apply(slot0Config);
                break;
            case 1:
                Slot1Configs slot1Config = new Slot1Configs();
                slot1Config.kP = constants.kP;
                slot1Config.kI = constants.kI;
                slot1Config.kD = constants.kD;
                slot1Config.kV = constants.kF;
                slot1Config.kS = constants.kS;
                m_configurator.apply(slot1Config);
                break;
            case 2:
                Slot2Configs slot2Config = new Slot2Configs();
                slot2Config.kP = constants.kP;
                slot2Config.kI = constants.kI;
                slot2Config.kD = constants.kD;
                slot2Config.kV = constants.kF;
                slot2Config.kS = constants.kS;
                m_configurator.apply(slot2Config);
                break;
            default:
                DriverStation.reportWarning(
                    "v6 TalonFX only supports slots 0, 1, and 2. Not applying PID configuration.", false);
                break;
        }
    }

    @Override
    public BeakPIDConstants getPID(int slot) {
        // The v6 slot API is wacky
        BeakPIDConstants constants = new BeakPIDConstants();
        m_configurator.refresh(m_config);
        switch (slot) {
            case 0:
                Slot0Configs slot0Config = m_config.Slot0;
                constants.kP = slot0Config.kP;
                constants.kI = slot0Config.kI;
                constants.kD = slot0Config.kD;
                constants.kF = slot0Config.kV;
                constants.kS = slot0Config.kS;
                break;
            case 1:
                Slot1Configs slot1Config = m_config.Slot1;
                constants.kP = slot1Config.kP;
                constants.kI = slot1Config.kI;
                constants.kD = slot1Config.kD;
                constants.kF = slot1Config.kV;
                constants.kS = slot1Config.kS;
                break;
            case 2:
                Slot2Configs slot2Config = m_config.Slot2;
                constants.kP = slot2Config.kP;
                constants.kI = slot2Config.kI;
                constants.kD = slot2Config.kD;
                constants.kF = slot2Config.kV;
                constants.kS = slot2Config.kS;
                break;
            default:
                DriverStation.reportWarning(
                    "v6 TalonFX only supports slots 0, 1, and 2. Returning blank PID configuration.", false);
                break;
        }
        return constants;
    }

    @Override
    public void setReverseLimitSwitchNormallyClosed(boolean normallyClosed) {
        HardwareLimitSwitchConfigs config = new HardwareLimitSwitchConfigs();
        m_configurator.refresh(config);

        config.ReverseLimitEnable = true;
        config.ReverseLimitType = normallyClosed ? ReverseLimitTypeValue.NormallyClosed
            : ReverseLimitTypeValue.NormallyOpen;
        config.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;

        m_configurator.apply(config);
    }

    @Override
    public void setForwardLimitSwitchNormallyClosed(boolean normallyClosed) {
        HardwareLimitSwitchConfigs config = new HardwareLimitSwitchConfigs();
        m_configurator.refresh(config);

        config.ForwardLimitEnable = true;
        config.ForwardLimitType = normallyClosed ? ForwardLimitTypeValue.NormallyClosed
            : ForwardLimitTypeValue.NormallyOpen;
        config.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;

        m_configurator.apply(config);
    }

    @Override
    public void setReverseExtremePosition(double nu) {
        HardwareLimitSwitchConfigs config = new HardwareLimitSwitchConfigs();
        m_configurator.refresh(config);

        config.ReverseLimitAutosetPositionEnable = true;
        config.ReverseLimitAutosetPositionValue = nu;

        m_configurator.apply(config);
    }

    @Override
    public void setForwardExtremePosition(double nu) {
        HardwareLimitSwitchConfigs config = new HardwareLimitSwitchConfigs();
        m_configurator.refresh(config);

        config.ForwardLimitAutosetPositionEnable = true;
        config.ForwardLimitAutosetPositionValue = nu;

        m_configurator.apply(config);
    }

    @Override
    public DataSignal<Boolean> getReverseLimitSwitch() {
        StatusSignal<ReverseLimitValue> value = super.getReverseLimit();
        boolean boolValue = value.getValue() == ReverseLimitValue.ClosedToGround;
        return new DataSignal<Boolean>(boolValue, value.getTimestamp().getTime());
    }

    @Override
    public DataSignal<Boolean> getForwardLimitSwitch() {
        StatusSignal<ForwardLimitValue> value = super.getForwardLimit();
        boolean boolValue = value.getValue() == ForwardLimitValue.ClosedToGround;
        return new DataSignal<Boolean>(boolValue, value.getTimestamp().getTime());
    }

    @Override
    public void setSupplyCurrentLimit(int amps) {
        CurrentLimitsConfigs config = new CurrentLimitsConfigs();
        m_configurator.refresh(config);

        config.SupplyCurrentLimitEnable = true;
        config.SupplyCurrentLimit = amps;
        config.SupplyCurrentThreshold = amps + 5.;
        config.SupplyTimeThreshold = 0.1;

        m_configurator.apply(config);
    }

    @Override
    public void setStatorCurrentLimit(int amps) {
        CurrentLimitsConfigs config = new CurrentLimitsConfigs();
        m_configurator.refresh(config);

        config.StatorCurrentLimitEnable = true;
        config.StatorCurrentLimit = amps;

        m_configurator.apply(config);
    }

    @Override
    public void restoreFactoryDefault() {
        m_config = new TalonFXConfiguration();
        m_configurator.apply(m_config);
    }

    @Override
    public void setAllowedClosedLoopError(double error, int slot) {
        MotorOutputConfigs config = new MotorOutputConfigs();
        m_configurator.refresh(config);

    }

    @Override
    public void setVoltageCompensationSaturation(double saturation) {
        m_voltageCompEnabled = saturation != 0.;
    }

    @Override
    public void setMotionMagicCruiseVelocity(double velocity, int slot) {
        MotionMagicConfigs config = new MotionMagicConfigs();
        m_configurator.refresh(config);

        config.MotionMagicCruiseVelocity = velocity;

        m_configurator.apply(config);
    }

    @Override
    public void setMotionMagicAcceleration(double accel, int slot) {
        MotionMagicConfigs config = new MotionMagicConfigs();
        m_configurator.refresh(config);

        config.MotionMagicAcceleration = accel;

        m_configurator.apply(config);
    }

    @Override
    public void set(double percentOutput, double arbFeedforward) {
        if (m_voltageCompEnabled) {
            super.setControl(m_voltageOut
            .withOutput(percentOutput * getSuppliedVoltage().Value + arbFeedforward));
        } else {
            super.setControl(m_dutyCycleOut
            .withOutput(percentOutput + arbFeedforward / 12.));
        }
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
