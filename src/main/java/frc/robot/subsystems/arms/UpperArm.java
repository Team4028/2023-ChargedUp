// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * The upper Argos Arm
 */
public class UpperArm extends Arm {
    private static final double kP = 0.2;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kIz = 0.0;
    private static final double kFF = 0.0;

    private static final double kMaxOutput = 0.7;
    private static final double kMinOutput = -0.7;

    private static final double kS = 0.; // 0.33069;
    private static final double kG = 0.; // 0.2554;
    private static final double kV = 0.; // 0.10667;

    private static final double ZEROING_VBUS = -0.1;
    private static final double ZEROING_CURRENT_THRESHOLD = 20.0;

    private static UpperArm m_instance;

    /*
     * Inches per revelution of sprocket: 6.25
     * Gear reduction 12:1
     */
    private static final double NATIVE_UNITS_TO_INCHES = 6.25 / 12;
    public final double maxVel, maxAccel;

    /** Creates a new UpperArm. */
    public UpperArm() {
        maxVel = 3500;// 7000;
        maxAccel = 5000;// 14000;

        ffmodel = new ElevatorFeedforward(kS, kG, kV);

        m_motor = new CANSparkMax(9, MotorType.kBrushless);
        m_motor.setInverted(true);

        m_pid = m_motor.getPIDController();

        m_pid.setP(kP);
        m_pid.setI(kI);
        m_pid.setD(kD);
        m_pid.setIZone(kIz);
        m_pid.setFF(kFF);

        m_pid.setOutputRange(kMinOutput, kMaxOutput);

        super.initArm();
    }

    public static UpperArm getInstance() {
        if (m_instance == null) {
            m_instance = new UpperArm();
        }
        return m_instance;
    }

    public void armTen() {
        m_pid.setReference(2, CANSparkMax.ControlType.kPosition/* change to kPosition to disable smartMotion */);
        m_targetPosition = 10;
    }

    public void armThirty() {
        m_pid.setReference(31.0451, CANSparkMax.ControlType.kPosition);
        m_targetPosition = 30;
    }

    public void armSixty() {
        m_pid.setReference(53.62151, CANSparkMax.ControlType.kPosition);
        m_targetPosition = 60;
    }

    public void armNintey() {
        m_pid.setReference(76.203, CANSparkMax.ControlType.kPosition);
        m_targetPosition = 90;
    }

    @Override
    public double nativeUnitsToInches(double nativeUntis) {
        return nativeUntis * NATIVE_UNITS_TO_INCHES;
    }

    @Override
    public double inchesToNativeUnits(double inches) {
        return inches / NATIVE_UNITS_TO_INCHES;
    }

    @Override
    public double getTargetPositionInches() {
        return m_targetPosition * NATIVE_UNITS_TO_INCHES;
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public CommandBase exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
                /* one-time action goes here */
            });
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a
     * digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("UpArmEncoderPos", nativeUnitsToInches(getEncoderPosition()));
        SmartDashboard.putNumber("UpArmErr", this.getError());
        SmartDashboard.putNumber("UpArmCurrentAmps", this.getMotorCurrent());
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    @Override
    public double getZeroCurrentThreshold() {
        return ZEROING_CURRENT_THRESHOLD;
    }

    @Override
    public double getZeroVbus() {
        return ZEROING_VBUS;
    }
}
