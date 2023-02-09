// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm2 extends SubsystemBase {
    private SparkMaxPIDController m_pid;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, maxAcc, minVel, allowedErr;
    private CANSparkMax m_armMotor;
    private RelativeEncoder m_encoder;
    private static Arm2 m_instance;
    private double pidPos=0;
    /** Creates a new ExampleSubsystem. */
    public Arm2() {
        m_armMotor = new CANSparkMax(10, MotorType.kBrushless);
        m_armMotor.setInverted(true);
        m_encoder = m_armMotor.getEncoder();
        m_armMotor.setSmartCurrentLimit(30);
        m_pid = m_armMotor.getPIDController();
        kP = .6;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0.000156;
        kMaxOutput = .9;
        kMinOutput = -.9;
        // smart motion cooefs
        maxVel = 5000 / 12;
        maxAcc = 5000 / 12;

        m_pid.setP(kP);
        m_pid.setI(kI);
        m_pid.setD(kD);
        m_pid.setIZone(kIz);
        m_pid.setFF(kFF);
        m_pid.setOutputRange(kMinOutput, kMaxOutput);
        // Smart Motion
        m_pid.setSmartMotionMaxVelocity(maxVel, 0);
        m_pid.setSmartMotionMinOutputVelocity(minVel, 0);
        m_pid.setSmartMotionMaxAccel(maxAcc, 0);
        m_pid.setSmartMotionAllowedClosedLoopError(allowedErr, 0);

    }

    public void runArm(double speed) {
        m_armMotor.set(speed);
    }

    public double getMotorCurrent() {
        return m_armMotor.getOutputCurrent();
    }

    public void armTen() {
        m_pid.setReference(3.09, CANSparkMax.ControlType.kPosition/* change to kPosition to disable smartMotion */);
        pidPos=10;
    }

    public void armThirty() {
        m_pid.setReference(11.33103, CANSparkMax.ControlType.kPosition);
        pidPos=10;
    }

    public void armSixty() {
        m_pid.setReference(19.56897, CANSparkMax.ControlType.kPosition);
        pidPos=60;
    }

    public void armNintey() {
        m_pid.setReference(27.81, CANSparkMax.ControlType.kPosition);
        pidPos=90;
    }

    public void setEncoderPosition(double position) {
        m_encoder.setPosition(position);
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

    public static Arm2 getInstance() {
        if (m_instance == null) {
            m_instance = new Arm2();
        }
        return m_instance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Encoder Pos2", m_encoder.getPosition());
        SmartDashboard.putNumber("pidPos",pidPos);
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
