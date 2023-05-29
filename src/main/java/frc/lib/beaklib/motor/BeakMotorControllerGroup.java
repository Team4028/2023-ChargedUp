// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.motor;

import frc.lib.beaklib.pid.BeakPIDConstants;
import frc.lib.beaklib.units.Distance;

/** A combined group of motor controllers to be controlled as one. */
public class BeakMotorControllerGroup implements BeakMotorController {
    private BeakMotorController[] m_controllers;

    private boolean m_inverted = false;

    public BeakMotorControllerGroup(BeakMotorController... motorControllers) {
        m_controllers = motorControllers;
    }

    @Override
    public void set(double speed) {
        for (BeakMotorController controller : m_controllers) {
            controller.set(speed);
        }
    }

    @Override
    public double get() {
        // all should have the same applied output
        return m_controllers[0].get();
    }

    @Override
    public void setInverted(boolean isInverted) {
        // This is an interesting implementation but one I'm keeping for various reasons.
        // Sometimes, a user will pass in two motor controllers, one is inverted, one isn't.
        // Say that, at some point, they need to invert this. It makes less sense to change
        // the inversion of each and more sense to call a function that will invert each motor
        // controller from its current state.

        m_inverted = isInverted;
        for (BeakMotorController controller : m_controllers) {
            // :)
            controller.setInverted(isInverted ^ controller.getInverted());
        }
    }

    @Override
    public boolean getInverted() {
        return m_inverted;
    }

    @Override
    public void setBrake(boolean brake) {
        for (BeakMotorController controller : m_controllers) {
            controller.setBrake(brake);
        }
    }

    @Override
    public void setVelocityNU(double nu, double arbFeedforward, int slot) {
        for (BeakMotorController controller : m_controllers) {
            controller.setVelocityNU(nu, arbFeedforward, slot);
        }
    }

    @Override
    public void setPositionNU(double nu, double arbFeedforward, int slot) {
        for (BeakMotorController controller : m_controllers) {
            controller.setPositionNU(nu, arbFeedforward, slot);
        }
    }

    @Override
    public void setEncoderPositionNU(double nu) {
        for (BeakMotorController controller : m_controllers) {
            controller.setEncoderPositionNU(nu);
        }
    }

    @Override
    public void setMotionMagicNU(double nu, double arbFeedforward, int slot) {
        for (BeakMotorController controller : m_controllers) {
            controller.setMotionMagicNU(nu, arbFeedforward, slot);
        }
    }

    @Override
    public DataSignal<Double> getVelocityNU() {
        return m_controllers[0].getVelocityNU();
    }

    @Override
    public DataSignal<Double> getPositionNU() {
        return m_controllers[0].getPositionNU();
    }

    @Override
    public DataSignal<Double> getSuppliedVoltage() {
        return m_controllers[0].getSuppliedVoltage();
    }

    @Override
    public void setPID(BeakPIDConstants constants, int slot) {
        for (BeakMotorController controller : m_controllers) {
            controller.setPID(constants, slot);
        }
    }

    @Override
    public BeakPIDConstants getPID(int slot) {
        return m_controllers[0].getPID(slot);
    }

    @Override
    public void setReverseLimitSwitchNormallyClosed(boolean normallyClosed) {
        for (BeakMotorController controller : m_controllers) {
            controller.setReverseLimitSwitchNormallyClosed(normallyClosed);
        }
    }

    @Override
    public void setForwardLimitSwitchNormallyClosed(boolean normallyClosed) {
        for (BeakMotorController controller : m_controllers) {
            controller.setForwardLimitSwitchNormallyClosed(normallyClosed);
        }
    }

    @Override
    public DataSignal<Boolean> getReverseLimitSwitch() {
        return m_controllers[0].getReverseLimitSwitch();
    }

    @Override
    public DataSignal<Boolean> getForwardLimitSwitch() {
        return m_controllers[0].getForwardLimitSwitch();
    }

    @Override
    public void setSupplyCurrentLimit(int amps) {
        for (BeakMotorController controller : m_controllers) {
            controller.setSupplyCurrentLimit(amps);
        }
    }

    @Override
    public void setStatorCurrentLimit(int amps) {
        for (BeakMotorController controller : m_controllers) {
            controller.setStatorCurrentLimit(amps);
        }
    }

    @Override
    public void restoreFactoryDefault() {
        for (BeakMotorController controller : m_controllers) {
            controller.restoreFactoryDefault();
        }
    }

    @Override
    public void setAllowedClosedLoopError(double error, int slot) {
        for (BeakMotorController controller : m_controllers) {
            controller.setAllowedClosedLoopError(error, slot);
        }
    }

    @Override
    public void setVoltageCompensationSaturation(double saturation) {
        for (BeakMotorController controller : m_controllers) {
            controller.setVoltageCompensationSaturation(saturation);
        }
    }

    @Override
    public void setMotionMagicCruiseVelocity(double velocity, int slot) {
        for (BeakMotorController controller : m_controllers) {
            controller.setMotionMagicCruiseVelocity(velocity, slot);
        }
    }

    @Override
    public void setMotionMagicAcceleration(double accel, int slot) {
        for (BeakMotorController controller : m_controllers) {
            controller.setMotionMagicAcceleration(accel, slot);
        }
    }

    @Override
    public void set(double percentOutput, double arbFeedforward) {
        for (BeakMotorController controller : m_controllers) {
            controller.set(percentOutput, arbFeedforward);
        }
    }

    @Override
    public void setStatusPeriod(int value, int period) {
        for (BeakMotorController controller : m_controllers) {
            controller.setStatusPeriod(value, period);
        }
    }

    @Override
    public void setVelocityConversionConstant(double constant) {
        for (BeakMotorController controller : m_controllers) {
            controller.setVelocityConversionConstant(constant);
        }
    }

    @Override
    public double getVelocityConversionConstant() {
        return m_controllers[0].getVelocityConversionConstant();
    }

    @Override
    public void setPositionConversionConstant(double constant) {
        for (BeakMotorController controller : m_controllers) {
            controller.setPositionConversionConstant(constant);
        }
    }

    @Override
    public double getPositionConversionConstant() {
        return m_controllers[0].getPositionConversionConstant();
    }

    @Override
    public void setEncoderGearRatio(double ratio) {
        for (BeakMotorController controller : m_controllers) {
            controller.setEncoderGearRatio(ratio);
        }
    }

    @Override
    public double getEncoderGearRatio() {
        return m_controllers[0].getEncoderGearRatio();
    }

    @Override
    public void setWheelDiameter(Distance diameter) {
        for (BeakMotorController controller : m_controllers) {
            controller.setWheelDiameter(diameter);
        }
    }

    @Override
    public Distance getWheelDiameter() {
        return m_controllers[0].getWheelDiameter();
    }

}
