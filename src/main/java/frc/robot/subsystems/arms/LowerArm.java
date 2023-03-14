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
 * The lower Argos Arm
 */
public class LowerArm extends Arm {
    private static final double kP = 0.175;
    private static final double kI = 0.0;
    private static final double kD = 1.6;
    private static final double kIz = 0.0;
    private static final double kFF = 0.0;

    private static final double kMaxOutput = 0.85;
    private static final double kMinOutput = -0.5;

    private static final double kS = 0.1; // 0.33069;
    private static final double kG = 0.1; // 0.2554;
    private static final double kV = 0.1; // 0.10667;

    public static final double ZEROING_VBUS = -0.1;
    public static final double ZEROING_CURRENT_THRESHOLD = 20.0;

    private static LowerArm m_instance;
    public final double maxVel, maxAccel;

    /** Creates a new ExampleSubsystem. */
    public LowerArm() {
        maxVel = 45.0; // RPS
        maxAccel = 90.0; // RPS^2

        ffmodel = new ElevatorFeedforward(kS, kG, kV);

        m_motor = new CANSparkMax(10, MotorType.kBrushless);
        m_motor.setInverted(true);

        m_pid = m_motor.getPIDController();
        m_pid.setP(kP);
        m_pid.setI(kI);
        m_pid.setD(kD);
        m_pid.setIZone(kIz);
        m_pid.setFF(kFF);

        m_pid.setOutputRange(kMinOutput, kMaxOutput);

        // TODO: initArm should take in the constants we have and configure directly.
        super.initArm();
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
        SmartDashboard.putNumber("LowArmEncoderPos", getEncoderPosition());
        SmartDashboard.putNumber("LowArmErr", this.getError());
        SmartDashboard.putNumber("LowArmCurrentAmps", this.getMotorCurrent());
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
