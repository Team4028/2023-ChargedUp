// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * The upper Argos Arm
 */
public class UpperArm extends Arm {
    private static final double kP = 0.11;//0.15;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kIz = 0.0;
    private static final double kFF = 0.0;

    private static final double kMaxOutput = 0.7;
    private static final double kMinOutput = -0.7;

    private static final double kS = 0.089792;
    private static final double kG = 0; //-0.22255;
    private static final double kV = 0.13158;

    private static final double ZEROING_VBUS = -0.15;
    private static final double ZEROING_CURRENT_THRESHOLD = 28.0;

    private static UpperArm m_instance;

    public final double MaxVel, MaxAccel;

    /** Creates a new UpperArm. */
    public UpperArm() {
        MaxVel = 80.0; // RPS
        MaxAccel = 160.0; // RPS^2

        FFModel = new ElevatorFeedforward(kS, kG, kV);

        m_motor = new CANSparkMax(9, MotorType.kBrushless);
        m_motor.setInverted(true);

        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 17);

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
        SmartDashboard.putNumber("Up Arm Pos", getEncoderPosition());
        SmartDashboard.putNumber("Up Arm Target", this.m_targetPosition);
        SmartDashboard.putNumber("Up Arm Err", this.getError());
        SmartDashboard.putNumber("Up Arm Amps", this.getMotorCurrent());
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
