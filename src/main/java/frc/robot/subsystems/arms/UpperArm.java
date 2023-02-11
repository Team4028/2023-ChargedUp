// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * The upper Argos Arm
 */
public class UpperArm extends Arm {

    private static UpperArm m_instance;

    /** Creates a new UpperArm. */
    public UpperArm() {
        m_motor = new CANSparkMax(9, MotorType.kBrushless);
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
        m_pidPos = 10;
    }

    public void armThirty() {
        m_pid.setReference(31.0451, CANSparkMax.ControlType.kPosition);
        m_pidPos = 30;
    }

    public void armSixty() {
        m_pid.setReference(53.62151, CANSparkMax.ControlType.kPosition);
        m_pidPos = 60;
    }

    public void armNintey() {
        m_pid.setReference(76.203, CANSparkMax.ControlType.kPosition);
        m_pidPos = 90;
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
        SmartDashboard.putNumber("UpArmEncoderPos", m_encoder.getPosition());
        SmartDashboard.putNumber("UpArmTargetPct", m_pidPos);
        SmartDashboard.putNumber("UpArmCurrentAmps", m_motor.getOutputCurrent());
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
