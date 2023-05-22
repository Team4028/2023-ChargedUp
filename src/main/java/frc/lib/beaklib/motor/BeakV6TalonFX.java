// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.motor;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class BeakV6TalonFX extends TalonFX implements BeakMotorController {
    private double m_distancePerPulse = 1.;

    private TalonFXConfigurator m_configurator;
    private TalonFXConfiguration m_config = new TalonFXConfiguration();

    private VelocityDutyCycle m_velocityOut = new VelocityDutyCycle(0.);
    private VelocityVoltage m_velocityVoltage = new VelocityVoltage(0.);
    private PositionDutyCycle m_positionOut = new PositionDutyCycle(0.);
    private PositionVoltage m_positionVoltage = new PositionVoltage(0.);
    private MotionMagicDutyCycle m_motionMagicOut = new MotionMagicDutyCycle(0.);
    private MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(0.);

    private boolean m_voltageCompEnabled = false;

    public BeakV6TalonFX(int port, String canBus) {
        super(port, canBus);
        m_configurator = super.getConfigurator();
        // super.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        // LimitSwitchNormal.NormallyOpen);
        // super.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        // LimitSwitchNormal.NormallyOpen);
    }

    public BeakV6TalonFX(int port) {
        this(port, "");
    }

    @Override
    public void setBrake(boolean brake) {
        // v6 is funky
        NeutralModeValue neutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        MotorOutputConfigs config = new MotorOutputConfigs();
        config.NeutralMode = neutralMode;
        m_configurator.apply(config, 0.1);
    }

    @Override
    public void setVelocityRPM(double rpm, double arbFeedforward, int slot) {
        setVelocityNU(rpm / 60., arbFeedforward, slot);
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
    public void setPositionMotorRotations(double rotations, double arbFeedforward, int slot) {
        setPositionNU(rotations, arbFeedforward, slot);
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
    public void setEncoderPositionMotorRotations(double rotations) {
        setEncoderPositionNU(rotations);
    }

    @Override
    public void setEncoderPositionNU(double nu) {
        super.setRotorPosition(nu, 0.1);
    }

    @Override
    public void setMotionMagicMotorRotations(double rotations, double arbFeedforward, int slot) {
        setMotionMagicNU(rotations, arbFeedforward, slot);
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
    public DataSignal<Double> getVelocityRPM() {
        DataSignal<Double> velocity = getVelocityNU();
        velocity.Value *= 60.;
        return velocity;
    }

    @Override
    public DataSignal<Double> getVelocityNU() {
        return new DataSignal<Double>(super.getVelocity());
    }

    @Override
    public DataSignal<Double> getPositionMotorRotations() {
        return getPositionNU();
    }

    @Override
    public DataSignal<Double> getPositionNU() {
        return new DataSignal<Double>(super.getPosition());
    }

    @Override
    public DataSignal<Double> getBusVoltage() {
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
    public double calculateFeedForward(double percentOutput, double desiredOutputNU) {
        throw new UnsupportedOperationException("Unimplemented method 'calculateFeedForward'");
    }

    @Override
    public double getVelocityEncoderCPR() {
        return 10.; // probably need to re-evaluate the CPR methods as a whole.
    }

    @Override
    public double getPositionEncoderCPR() {
        return 1.;
    }

    @Override
    public void setReverseLimitSwitchNormallyClosed(boolean normallyClosed) {
        
    }

    @Override
    public void setForwardLimitSwitchNormallyClosed(boolean normallyClosed) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setForwardLimitSwitchNormallyClosed'");
    }

    @Override
    public boolean getReverseLimitSwitch() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getReverseLimitSwitch'");
    }

    @Override
    public boolean getForwardLimitSwitch() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getForwardLimitSwitch'");
    }

    @Override
    public void setSupplyCurrentLimit(int amps) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSupplyCurrentLimit'");
    }

    @Override
    public void setStatorCurrentLimit(int amps) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setStatorCurrentLimit'");
    }

    @Override
    public void restoreFactoryDefault() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'restoreFactoryDefault'");
    }

    @Override
    public void setAllowedClosedLoopError(double error, int slot) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setAllowedClosedLoopError'");
    }

    @Override
    public void setVoltageCompensationSaturation(double saturation) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVoltageCompensationSaturation'");
    }

    @Override
    public void setMotionMagicCruiseVelocity(double velocity, int slot) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setMotionMagicCruiseVelocity'");
    }

    @Override
    public void setMotionMagicAcceleration(double accel, int slot) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setMotionMagicAcceleration'");
    }

    @Override
    public void set(double percentOutput, double arbFeedforward) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'set'");
    }

    @Override
    public void setStatusPeriod(int value, int period) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setStatusPeriod'");
    }

    @Override
    public void setDistancePerPulse(double dpr) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setDistancePerPulse'");
    }

    @Override
    public double getDistancePerPulse() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getDistancePerPulse'");
    }
}
