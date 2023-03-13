// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LEDs extends SubsystemBase {
    public enum CANdleMode {
        VICTORY_SPIN("VICTORY_SPIN"), FIRE("FIRE"), IDLE("IDLE"), ACTIVE("ACTIVE");

        public String name;

        private CANdleMode(String name) {
            this.name = name;
        }
    }

    private CANdleMode m_currentMode;
    private Color m_color;
    private final int NUM_LEDS = 128; // 8 for candle + 120 for 2 strips
    private CANdle m_candle;
    // private int r, g, b;
    private static LEDs m_instance;
    // private Timer scrollTimer = new Timer();
    private double scrollVar = 0;

    /**
     * the colors that the CANdle needs to be set to
     */
    public enum Color {
        GREEN(0, 254, 0), PURPLE(118, 0, 254), ORANGE(254, 55, 0), OFF(0, 0, 0);

        private int r;
        private int g;
        private int b;

        private Color(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }

    /** Creates a new LEDs. */
    public LEDs() {
        m_currentMode = CANdleMode.ACTIVE;
        m_candle = new CANdle(21, "rio");
        m_candle.configBrightnessScalar(0.5);
        m_candle.configLEDType(LEDStripType.GRB);
        m_candle.configLOSBehavior(true);

        setColor(Color.OFF);
    }

    /**
     * sets the color of the LED class
     * 
     * @param color
     *            the color to set
     */
    public void setColor(Color color) {
        m_color = color;
    }

    /**
     * sets the color to blank and runs {@code setLEDs()}
     */
    public void setBlank() {
        setColor(Color.OFF);
        // setLEDs().ignoringDisable(true).schedule();
    }

    /**
     * sets the color to the climb color (green)
     */
    public void setClimb() {
        setColor(Color.GREEN);
    }

    /**
     * sets the color to the cone color (yellow)
     */
    public void setCone() {
        setColor(Color.ORANGE);
    }

    /**
     * sets the color to the cube color (purple)
     */
    public void setCube() {
        setColor(Color.PURPLE);
    }

    public static LEDs getInstance() {
        if (m_instance == null) {
            m_instance = new LEDs();
        }
        return m_instance;
    }

    public SequentialCommandGroup blink(Color color) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> setBlank()),
            new WaitCommand(0.1),
            new InstantCommand(() -> setColor(color)),
            new WaitCommand(0.08),
            new InstantCommand(() -> setBlank()),
            new WaitCommand(0.08),
            new InstantCommand(() -> setColor(color)),
            new WaitCommand(0.08),
            new InstantCommand(() -> setBlank()),
            new WaitCommand(0.08),
            new InstantCommand(() -> setColor(color)),
            new WaitCommand(0.08),
            new InstantCommand(() -> setBlank()),
            new WaitCommand(0.08),
            new InstantCommand(() -> setColor(color)));
    }

    public void setIdle() {
        m_currentMode = CANdleMode.IDLE;
        m_candle.clearAnimation(0);
        m_candle.clearAnimation(1);
        m_candle.animate(new LarsonAnimation(Color.PURPLE.r, Color.PURPLE.g, Color.PURPLE.b, 0, 0.2, NUM_LEDS,
            BounceMode.Front, 8, 0), 0);
        m_candle.animate(new LarsonAnimation(Color.ORANGE.r, Color.ORANGE.g, Color.ORANGE.b, 0, 0.2, NUM_LEDS,
            BounceMode.Front, 8, 8), 1);
        m_candle.animate(new LarsonAnimation(Color.PURPLE.r, Color.PURPLE.g, Color.PURPLE.b, 0, 0.2, NUM_LEDS,
            BounceMode.Front, 8, 16), 2);
        m_candle.animate(new LarsonAnimation(Color.ORANGE.r, Color.ORANGE.g, Color.ORANGE.b, 0, 0.2, NUM_LEDS,
            BounceMode.Front, 8, 24), 3);
    }

    public void setVictorySpin() {
        // scrollTimer.restart();
        // scrollTimer.stop();
        m_currentMode = CANdleMode.VICTORY_SPIN;
        for (int i = 0; i < 4; i++) {
            m_candle.clearAnimation(i);
        }
        m_candle.animate(new RainbowAnimation(1, 1, NUM_LEDS), 0);
    }

    public void setFire() {
        m_currentMode = CANdleMode.FIRE;
        for (int i = 0; i < 4; i++) {
            m_candle.clearAnimation(i);
        }
        m_candle.animate(new FireAnimation(1.0, 0.2, NUM_LEDS - 60, 0.6, 0.4, true, 0), 0);
        m_candle.animate(new FireAnimation(1.0, 0.2, NUM_LEDS, 0.6, 0.4, false, NUM_LEDS - 60), 1);
    }

    public void setActive() {
        // scrollTimer.restart();
        // scrollTimer.stop();
        m_currentMode = CANdleMode.ACTIVE;
        for (int i = 0; i < 4; i++) {
            m_candle.clearAnimation(i);
        }
    }

    public CANdleMode getMode() {
        return m_currentMode;
    }

    public Color getColor() {
        return m_color;
    }

    @Override
    public void periodic() {
        // setLEDs().schedule();
        SmartDashboard.putString("Mode: ", m_currentMode.name);
        // SmartDashboard.putNumber("Timer", scrollTimer.get());
        SmartDashboard.putNumber("scrollVar", scrollVar);
        if (m_currentMode == CANdleMode.ACTIVE) {
            m_candle.setLEDs(m_color.r, m_color.g, m_color.b);
        }
    }
}