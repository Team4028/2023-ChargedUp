// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.arms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * The lower Argos Arm
 */
public class LowerArm extends Arm {
    private static LowerArm m_instance;
    private final double kS = 0.33069;
    private final double kG = 0.2554;
    private final double kV = 0.10667;

    /** Creates a new ExampleSubsystem. */
    public LowerArm() {
        ffmodel = new ElevatorFeedforward(kS, kG, kV);
        m_motor = new CANSparkMax(10, MotorType.kBrushless);
        m_motor.setInverted(true);
        super.initArm();
    }

    public void armTen() {
        m_pid.setReference(3.09, CANSparkMax.ControlType.kPosition/* change to kPosition to disable smartMotion */);
        m_pidPos = 10;
    }

    public void armThirty() {
        m_pid.setReference(11.33103, CANSparkMax.ControlType.kPosition);
        m_pidPos = 10;
    }

    public void armSixty() {
        m_pid.setReference(19.56897, CANSparkMax.ControlType.kPosition);
        m_pidPos = 60;
    }

    public void armNintey() {
        m_pid.setReference(27.81, CANSparkMax.ControlType.kPosition);
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
        return runOnce(() -> {
            /* one-time action goes here */
        });
        // THIS IS A CURSED WAY TO DO 2 COMMANDS
        // return new SequentialCommandGroup(runOnce(()->{
        // //run wrist
        // }),runOnce(()->{
        // //run gripper
        // }));
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

    public static LowerArm getInstance() {
        if (m_instance == null) {
            m_instance = new LowerArm();
        }
        return m_instance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("LowArmEncoderPos", this.getEncoderPosition());
        SmartDashboard.putNumber("LowArmErr", this.getError());
        SmartDashboard.putNumber("LowArmCurrentAmps", this.getMotorCurrent());
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
