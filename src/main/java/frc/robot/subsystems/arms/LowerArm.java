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
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kIz = 0.0;
    private static final double kFF = 0.0;

    private static final double kMaxOutput = 0.2;
    private static final double kMinOutput = -0.2;

    private static final double kS = 0.33069;
    private static final double kG = 0.2554;
    private static final double kV = 0.10667;

    private static LowerArm m_instance;
    public final double maxVel, maxAccel;

    /*
     * Inches per revelution of sprocket: 6.25
     * Gear reduction 25:1
     */
    public static final double NATIVE_UNITS_TO_INCHES = 6.25 / 25;

    /** Creates a new ExampleSubsystem. */
    public LowerArm() {
        maxVel = 3500;// 7000;
        maxAccel = 3500;// 14000;

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
        return m_pidPos * NATIVE_UNITS_TO_INCHES;
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
        SmartDashboard.putNumber("LowArmEncoderPos", nativeUnitsToInches(getEncoderPosition()));
        SmartDashboard.putNumber("LowArmErr", this.getError());
        SmartDashboard.putNumber("LowArmCurrentAmps", this.getMotorCurrent());
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
