// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxLimitSwitch.Type;

/** Common motor controller interface for REV Spark MAX. */
public class BeakSparkMAX extends CANSparkMax implements BeakMotorController {
    private RelativeEncoder encoder;
    private SparkMaxPIDController pid;

    private SparkMaxLimitSwitch revLimitSwitch;
    private SparkMaxLimitSwitch fwdLimitSwitch;

    private double m_distancePerPulse;

    public BeakSparkMAX(int port) {
        super(port, MotorType.kBrushless);

        resetControllers();
    }

    private void resetControllers() {
        encoder = super.getEncoder();
        pid = super.getPIDController();

        revLimitSwitch = super.getReverseLimitSwitch(Type.kNormallyOpen);
        fwdLimitSwitch = super.getForwardLimitSwitch(Type.kNormallyOpen);
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
        pid.setReference(nu, ControlType.kVelocity, slot, arbFeedforward);
    }

    @Override
    public void setPositionMotorRotations(double rotations, double arbFeedforward, int slot) {
        setPositionNU(rotations * encoder.getCountsPerRevolution(), arbFeedforward, slot);
    }

    @Override
    public void setPositionNU(double nu, double arbFeedforward, int slot) {
        pid.setReference(nu, ControlType.kPosition, slot, arbFeedforward);
    }

    @Override
    public void setEncoderPositionMotorRotations(double rotations) {
        setEncoderPositionNU(rotations * encoder.getCountsPerRevolution());
    }

    @Override
    public void setEncoderPositionNU(double nu) {
        encoder.setPosition(nu);
    }

    @Override
    public void setMotionMagicMotorRotations(double rotations, double arbFeedforward, int slot) {
        setMotionMagicNU(rotations, arbFeedforward, slot);
    }

    @Override
    public void setMotionMagicNU(double nu, double arbFeedforward, int slot) {
        pid.setReference(nu, ControlType.kSmartMotion, slot, arbFeedforward);
    }

    @Override
    public double getVelocityRPM() {
        return getVelocityNU();
    }

    @Override
    public double getVelocityNU() {
        return encoder.getVelocity();
    }

    @Override
    public double getPositionMotorRotations() {
        return getPositionNU() / encoder.getCountsPerRevolution();
    }

    @Override
    public double getPositionNU() {
        return encoder.getPosition();
    }

    @Override
    public double getP(int slot) {
        return pid.getP(slot);
    }

    @Override
    public double getI(int slot) {
        return pid.getI(slot);
    }

    @Override
    public double getD(int slot) {
        return pid.getD(slot);
    }

    @Override
    public double getF(int slot) {
        return pid.getFF(slot);
    }

    @Override
    public void setP(double p, int slot) {
        pid.setP(p, slot);
    }

    @Override
    public void setI(double i, int slot) {
        pid.setI(i, slot);
    }

    @Override
    public void setD(double d, int slot) {
        pid.setD(d, slot);
    }

    @Override
    public void setF(double f, int slot) {
        pid.setFF(f, slot);
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
    public double getOutputVoltage() {
        return super.getAppliedOutput() * super.getBusVoltage();
    }

    @Override
    public void setReverseLimitSwitchNormallyClosed(boolean normallyClosed) {
        revLimitSwitch = super.getReverseLimitSwitch(normallyClosed ? Type.kNormallyClosed : Type.kNormallyOpen);
    }

    @Override
    public void setForwardLimitSwitchNormallyClosed(boolean normallyClosed) {
        fwdLimitSwitch = super.getForwardLimitSwitch(normallyClosed ? Type.kNormallyClosed : Type.kNormallyOpen);
    }

    @Override
    public boolean getReverseLimitSwitch() {
        return revLimitSwitch.isPressed();
    }

    @Override
    public boolean getForwardLimitSwitch() {
        return fwdLimitSwitch.isPressed();
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
        pid.setSmartMotionAllowedClosedLoopError(error, slot);
    }

    @Override
    public void setVoltageCompensationSaturation(double saturation) {
        super.enableVoltageCompensation(saturation);
    }

    @Override
    public void setMotionMagicAcceleration(double accel, int slot) {
        pid.setSmartMotionMaxAccel(accel, slot);
    }

    @Override
    public void setMotionMagicCruiseVelocity(double velocity, int slot) {
        pid.setSmartMotionMaxVelocity(velocity, slot);
    }

    @Override
    public void set(double percentOutput, double arbFeedforward) {
        pid.setReference(percentOutput, ControlType.kDutyCycle, 0, arbFeedforward);
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
}
