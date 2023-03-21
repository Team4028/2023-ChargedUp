// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.OneMechanism;
import frc.robot.commands.LarsonAnimationV2;
import frc.robot.commands.LarsonAnimationV2.V2BounceMode;

public class LEDs extends SubsystemBase {
    public enum CANdleMode {
        VICTORY_SPIN, FIRE, IDLE, ACTIVE, IDLEV2;
    }

    private CANdleMode m_currentMode;
    private Color m_color, m_lastColor;
    private final int NUM_LEDS = 119;
    private CANdle m_candle;
    private boolean m_throwOnGround = false;
    // private int r, g, b;
    private static LEDs m_instance;
    // private Timer scrollTimer = new Timer();
    // private double scrollVar = 0;

    /**
     * the colors that the CANdle needs to be set to
     */
    public enum Color {
        GREEN(0, 254, 0), PURPLE(118, 0, 254), ORANGE(254, 55, 0), WHITE(255, 255, 255), OFF(0, 0, 0);

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

    /**
     * @param color
     *            the color to blink
     * @return Blinks the color and then changes to it (SequentialCommandGroup)
     */
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

    /**
     * @return Blinks the lights at their current color (SequentialCommandGroup)
     */
    public SequentialCommandGroup blink() {
        m_lastColor = m_color;
        return new SequentialCommandGroup(
            new InstantCommand(() -> setBlank()),
            new WaitCommand(0.1),
            new InstantCommand(() -> setColor(m_lastColor)),
            new WaitCommand(0.08),
            new InstantCommand(() -> setBlank()),
            new WaitCommand(0.08),
            new InstantCommand(() -> setColor(m_lastColor)),
            new WaitCommand(0.08),
            new InstantCommand(() -> setBlank()),
            new WaitCommand(0.08),
            new InstantCommand(() -> setColor(m_lastColor)),
            new WaitCommand(0.08),
            new InstantCommand(() -> setBlank()),
            new WaitCommand(0.08),
            new InstantCommand(() -> setColor(m_lastColor)));
    }

    /**
     * @param color
     *            The color to blink
     * @return Blinks the color, but returns to the original color
     */
    public SequentialCommandGroup blinkWithoutChange(Color color) {
        m_lastColor = m_color;
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
            new InstantCommand(() -> setBlank()),
            new WaitCommand(0.08),
            new InstantCommand(() -> setColor(color)),
            new WaitCommand(0.08),
            new InstantCommand(() -> setBlank()),
            new WaitCommand(0.08),
            new InstantCommand(() -> setColor(m_lastColor)));
    }

    public SequentialCommandGroup blinkMulti(Color... colors) {
        SequentialCommandGroup cmd = new SequentialCommandGroup(
            new InstantCommand(() -> setBlank()),
            new WaitCommand(0.1),
            new InstantCommand(() -> setColor(colors[0])),
            new WaitCommand(0.08));
        if (colors.length > 1) {
            for (int i = 1; i < colors.length - 1; i++) {
                Color tempColor = colors[i];
                cmd.addCommands(
                    new InstantCommand(() -> setBlank()),
                    new WaitCommand(0.08),
                    new InstantCommand(() -> setColor(tempColor)),
                    new WaitCommand(0.08));
            }
            cmd.addCommands(
                new InstantCommand(() -> setBlank()),
                new WaitCommand(0.08),
                new InstantCommand(() -> setColor(colors[colors.length - 1])));
        }
        return cmd;
    }

    // TODO: Test \/
    public void setIdleV2() {
        m_currentMode = CANdleMode.IDLEV2;
        for (int i = 0; i < 8; i++) {
            m_candle.clearAnimation(i);
        }
        new LarsonAnimationV2(Color.PURPLE.r, Color.PURPLE.g, Color.PURPLE.b, 0, 0.2, false, 59, 8, 8, 8,
            V2BounceMode.FRONT, m_instance)
                .alongWith(
                    new LarsonAnimationV2(Color.ORANGE.r, Color.ORANGE.g, Color.ORANGE.b, 0, 0.2, false, 59, 8, 16, 8,
                        V2BounceMode.FRONT, m_instance))
                .alongWith(
                    new LarsonAnimationV2(Color.PURPLE.r, Color.PURPLE.g, Color.PURPLE.b, 0, 0.2, true, 119, 68, 119,
                        8,
                        V2BounceMode.FRONT, m_instance))
                .alongWith(
                    new LarsonAnimationV2(Color.ORANGE.r, Color.ORANGE.g, Color.ORANGE.b, 0, 0.2, true, 119, 68, 111,
                        8,
                        V2BounceMode.FRONT, m_instance))
                .schedule();
    }

    public void setIdle() {
        m_currentMode = CANdleMode.IDLE;
        for (int i = 0; i < 8; i++) {
            m_candle.clearAnimation(i);
        }
        // TODO: Make it Symmetric - IDLEV2 fixes this if it works
        // m_candle.animate(new LarsonAnimation(Color.PURPLE.r, Color.PURPLE.g,
        // Color.PURPLE.b, 0, 0.2, NUM_LEDS,
        // BounceMode.Front, 8, 0), 0);
        // m_candle.animate(new LarsonAnimation(Color.ORANGE.r, Color.ORANGE.g,
        // Color.ORANGE.b, 0, 0.2, NUM_LEDS,
        // BounceMode.Front, 8, 8), 1);
        // m_candle.animate(new LarsonAnimation(Color.PURPLE.r, Color.PURPLE.g,
        // Color.PURPLE.b, 0, 0.2, NUM_LEDS,
        // BounceMode.Front, 8, 16), 2);
        // m_candle.animate(new LarsonAnimation(Color.ORANGE.r, Color.ORANGE.g,
        // Color.ORANGE.b, 0, 0.2, NUM_LEDS,
        // BounceMode.Front, 8, 24), 3);
        new SequentialCommandGroup(
            new InstantCommand(
                () -> m_candle.animate(new LarsonAnimation(Color.PURPLE.r, Color.PURPLE.g, Color.PURPLE.b, 0, 0.2, 51,
                    BounceMode.Front, 8, 8), 0)),
            new WaitCommand(0.1),
            new InstantCommand(
                () -> m_candle.animate(new LarsonAnimation(Color.ORANGE.r, Color.ORANGE.g, Color.ORANGE.b, 0, 0.2, 51,
                    BounceMode.Front, 8, 8), 1)),
            new WaitCommand(0.1),
            new InstantCommand(
                () -> m_candle.animate(new LarsonAnimation(Color.PURPLE.r, Color.PURPLE.g, Color.PURPLE.b, 0, 0.2, 51,
                    BounceMode.Front, 8, 8), 2)),
            new WaitCommand(0.1),
            new InstantCommand(
                () -> m_candle.animate(new LarsonAnimation(Color.ORANGE.r, Color.ORANGE.g, Color.ORANGE.b, 0, 0.2, 51,
                    BounceMode.Front, 8, 8), 3))).alongWith(new SequentialCommandGroup(
                        new InstantCommand(() -> m_candle
                            .animate(new LarsonAnimation(Color.PURPLE.r, Color.PURPLE.g, Color.PURPLE.b, 0, 0.2, 51,
                                BounceMode.Front, 8, 69), 4)),
                        new WaitCommand(0.1),
                        new InstantCommand(() -> m_candle
                            .animate(new LarsonAnimation(Color.ORANGE.r, Color.ORANGE.g, Color.ORANGE.b, 0, 0.2, 51,
                                BounceMode.Front, 8, 69), 5)),
                        new WaitCommand(0.1),
                        new InstantCommand(() -> m_candle
                            .animate(new LarsonAnimation(Color.PURPLE.r, Color.PURPLE.g, Color.PURPLE.b, 0, 0.2, 51,
                                BounceMode.Front, 8, 69), 6)),
                        new WaitCommand(0.1),
                        new InstantCommand(() -> m_candle
                            .animate(new LarsonAnimation(Color.ORANGE.r, Color.ORANGE.g, Color.ORANGE.b, 0, 0.2, 51,
                                BounceMode.Front, 8, 69), 7))))
                        .schedule();
    }

    public void setVictorySpin() {
        // scrollTimer.restart();
        // scrollTimer.stop();
        m_currentMode = CANdleMode.VICTORY_SPIN;
        for (int i = 0; i < 8; i++) {
            m_candle.clearAnimation(i);
        }
        m_candle.animate(new RainbowAnimation(1, 1, NUM_LEDS), 0);
    }

    public void setFire() {
        m_currentMode = CANdleMode.FIRE;
        for (int i = 0; i < 8; i++) {
            m_candle.clearAnimation(i);
        }
        m_candle.animate(new FireAnimation(1.0, 0.01, 51, 0.6, 0.3, true, 8), 0);
        m_candle.animate(new FireAnimation(1.0, 0.1, 51, 0.6, 0.3, false, NUM_LEDS - 42), 1);
    }

    public void setActive() {
        // scrollTimer.restart();
        // scrollTimer.stop();
        m_currentMode = CANdleMode.ACTIVE;
        for (int i = 0; i < 8; i++) {
            m_candle.clearAnimation(i);
        }
    }

    public void toggleThrowOnGround() {
        m_throwOnGround = !m_throwOnGround;
    }

    public CANdleMode getMode() {
        return m_currentMode;
    }

    public Color getColor() {
        return m_color;
    }

    public CANdle getCandle() {
        return m_candle;
    }

    public int getNumLEDs() {
        return NUM_LEDS;
    }

    @Override
    public void periodic() {
        // setLEDs().schedule();
        SmartDashboard.putString("Mode: ", m_currentMode.name());
        // SmartDashboard.putNumber("Timer", scrollTimer.get());
        // SmartDashboard.putNumber("scrollVar", scrollVar);
        if (m_currentMode == CANdleMode.ACTIVE) {
            if (OneMechanism.getAutoAlignMode() && !blink().isScheduled()) {
                blink().schedule();
            }
            if (m_throwOnGround) {
                m_candle.animate(new StrobeAnimation(m_color.r, m_color.g, m_color.b, 0, 1e-5, NUM_LEDS), 0);
            } else {
                m_candle.setLEDs(m_color.r, m_color.g, m_color.b);
            }
        } else if (m_currentMode == CANdleMode.FIRE) {
            if (m_throwOnGround) {
                m_candle.animate(new StrobeAnimation(m_color.r, m_color.g, m_color.b, 0, 1e-5, 8), 0);
            } else {
                m_candle.setLEDs(m_color.r, m_color.g, m_color.b, 0, 0, 8);
            }
        }
    }
}