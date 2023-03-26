// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.OneMechanism;
import frc.robot.commands.LEDs.LarsonAnimationV2;
import frc.robot.commands.LEDs.SlowAlternateBlink;
import frc.robot.commands.LEDs.LarsonAnimationV2.V2BounceMode;

public class LEDs extends SubsystemBase {

    /**
     * The mode that the CANdle display is in.
     */
    public enum CANdleMode {
        VICTORY_SPIN, //
        FIRE, //
        IDLE, //
        ACTIVE, //
        IDLEV2, //
        SLIDE;
    }

    private CANdleMode m_currentMode;
    private Color m_color, m_lastColor;
    private final int NUM_LEDS = 119;
    private final int STRIP_LENGTH = 51;
    private CANdle m_candle;
    private boolean m_throwOnGround = false;
    private boolean m_signal = false;
    private boolean m_fade = false;
    private static LEDs m_instance;
    /**
     * the colors that the CANdle needs to be set to
     */
    public enum Color {
        GREEN(0, 254, 0), //
        PURPLE(118, 0, 254), //
        ORANGE(254, 55, 0), //
        BLUE(0, 0, 254), //
        WHITE(254, 254, 254), //
        RED(254, 0, 0), //
        OFF(0, 0, 0);

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
        setColor(Color.ORANGE);
    }

    /**
     * Sets the color of the LEDs.
     * 
     * @param color
     *            the color to set
     */
    public void setColor(Color color) {
        m_lastColor = m_color;
        m_color = color;
    }

    /**
     * sets the color to blank
     */
    public void setBlank() {
        setColor(Color.OFF);
    }

    public SlowAlternateBlink setClimb() {
        return new SlowAlternateBlink(m_color, Color.GREEN, 0.5, m_instance);
    }

    public static LEDs getInstance() {
        if (m_instance == null) {
            m_instance = new LEDs();
        }
        return m_instance;
    }

    public SequentialCommandGroup blink() {
        return blink(m_color);
    }

    /**
     * @return Blinks the lights at their current color (SequentialCommandGroup)
     */
    public SequentialCommandGroup blink(Color color) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> setBlank()),
            new WaitCommand(0.02),
            new InstantCommand(() -> setColor(color)),
            new WaitCommand(0.02),
            new InstantCommand(() -> setBlank()),
            new WaitCommand(0.02),
            new InstantCommand(() -> setColor(color)));
    }

    /**
     * @param color
     *            The color to blink
     * @return Blinks between the inputted color and the original color
     */
    public SequentialCommandGroup alternateBlink(Color color) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> setColor(color)),
            new WaitCommand(0.02),
            new InstantCommand(() -> setColor(m_lastColor)),
            new WaitCommand(0.02),
            new InstantCommand(() -> setColor(color)),
            new WaitCommand(0.02),
            new InstantCommand(() -> setColor(m_lastColor)),
            new WaitCommand(0.02));
    }

    /**
     * @param iterations
     *            the amount of times to blink
     * @param colors
     *            the colors to blink with
     * @return
     *         <p>
     *         a SequentialCommandGroup that
     *         </p>
     *         <p>
     *         blinks the colors specified for the amount of times specified.
     *         </p>
     */
    public SequentialCommandGroup blinkMulti(int iterations, Color... colors) {
        SequentialCommandGroup cmd = new SequentialCommandGroup();
        for (int i = 0; i < iterations; i++) {
            for (int j = 0; j < colors.length; j++) {
                Color tempColor = colors[j];
                cmd.addCommands(
                    runOnce(() -> setBlank()),
                    new WaitCommand(0.08),
                    runOnce(() -> setColor(tempColor)),
                    new WaitCommand(0.08));
            }
        }
        return cmd;
    }

    public void setBeacon(Color color) {
        m_candle.setLEDs(color.r, color.g, color.b, 0, 8, 8);
        m_candle.setLEDs(color.r, color.g, color.b, 0, NUM_LEDS - 8, 8);
    }

    /**
     * Toggles a fade animation with the given color
     * 
     * @param color
     *            If fade is true, the leds will fade with this color
     * @param fade
     *            whether to start (true) or cancel (false) the animation
     */
    public void setFade(Color color, boolean fade) {
        for (int i = 0; i < 8; i++) {
            m_candle.clearAnimation(i);
        }
        m_fade = fade;
        if (fade) {
            m_candle.animate(new SingleFadeAnimation(color.r, color.g, color.b, 0, 0.5, NUM_LEDS), 0);
        } else {
            m_candle.clearAnimation(0);
        }
    }

    public boolean getFade() {
        return m_fade;
    }

    // TODO: Test \/
    public void setIdleV2() {
        m_throwOnGround = false;
        m_currentMode = CANdleMode.IDLEV2;
        for (int i = 0; i < 8; i++) {
            m_candle.clearAnimation(i);
        }
        new LarsonAnimationV2(Color.PURPLE.r, Color.PURPLE.g, Color.PURPLE.b, 0, 0.2, false, 59, 8, 8, 8,
            V2BounceMode.FRONT, this)
                .alongWith(
                    new LarsonAnimationV2(Color.ORANGE.r, Color.ORANGE.g, Color.ORANGE.b, 0, 0.2, false, 59, 8, 16, 8,
                        V2BounceMode.FRONT, this))
                .alongWith(
                    new LarsonAnimationV2(Color.PURPLE.r, Color.PURPLE.g, Color.PURPLE.b, 0, 0.2, true, 119, 68, 119,
                        8,
                        V2BounceMode.FRONT, this))
                .alongWith(
                    new LarsonAnimationV2(Color.ORANGE.r, Color.ORANGE.g, Color.ORANGE.b, 0, 0.2, true, 119, 68, 111,
                        8,
                        V2BounceMode.FRONT, this))
                .schedule();
    }

    public void setSlide() {
        m_throwOnGround = false;
        m_currentMode = CANdleMode.SLIDE;
        for (int i = 0; i < 8; i++) {
            m_candle.clearAnimation(i);
        }
        m_candle.animate(
            new ColorFlowAnimation(m_color.r, m_color.g, m_color.b, 0, 0.5, NUM_LEDS, Direction.Forward, 8),
            0);
        m_candle.animate(new ColorFlowAnimation(m_color.r, m_color.g, m_color.b, 0, 0.5, NUM_LEDS,
            Direction.Backward), 1);
    }

    public void setIdle() {
        m_throwOnGround = false;
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
                () -> m_candle.animate(new LarsonAnimation(Color.PURPLE.r, Color.PURPLE.g, Color.PURPLE.b, 0, 0.2, STRIP_LENGTH,
                    BounceMode.Front, 8, 8), 0)),
            new WaitCommand(0.1),
            new InstantCommand(
                () -> m_candle.animate(new LarsonAnimation(Color.ORANGE.r, Color.ORANGE.g, Color.ORANGE.b, 0, 0.2, STRIP_LENGTH,
                    BounceMode.Front, 8, 16), 1)),
            new WaitCommand(0.1),
            new InstantCommand(
                () -> m_candle.animate(new LarsonAnimation(Color.PURPLE.r, Color.PURPLE.g, Color.PURPLE.b, 0, 0.2, STRIP_LENGTH,
                    BounceMode.Front, 8, 24), 2)),
            new WaitCommand(0.1),
            new InstantCommand(
                () -> m_candle.animate(new LarsonAnimation(Color.ORANGE.r, Color.ORANGE.g, Color.ORANGE.b, 0, 0.2, STRIP_LENGTH,
                    BounceMode.Front, 8, 32), 3))).alongWith(new SequentialCommandGroup(
                        new InstantCommand(() -> m_candle
                            .animate(new LarsonAnimation(Color.PURPLE.r, Color.PURPLE.g, Color.PURPLE.b, 0, 0.2, NUM_LEDS,
                                BounceMode.Front, 8, 69), 4)),
                        new WaitCommand(0.1),
                        new InstantCommand(() -> m_candle
                            .animate(new LarsonAnimation(Color.ORANGE.r, Color.ORANGE.g, Color.ORANGE.b, 0, 0.2, NUM_LEDS,
                                BounceMode.Front, 8, 77), 5)),
                        new WaitCommand(0.1),
                        new InstantCommand(() -> m_candle
                            .animate(new LarsonAnimation(Color.PURPLE.r, Color.PURPLE.g, Color.PURPLE.b, 0, 0.2, NUM_LEDS,
                                BounceMode.Front, 8, 85), 6)),
                        new WaitCommand(0.1),
                        new InstantCommand(() -> m_candle
                            .animate(new LarsonAnimation(Color.ORANGE.r, Color.ORANGE.g, Color.ORANGE.b, 0, 0.2, NUM_LEDS,
                                BounceMode.Front, 8, 93), 7))))
                        .schedule();
    }

    public void setVictorySpin() {
        m_throwOnGround = false;
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
        m_candle.animate(new FireAnimation(1.0, 0.1, NUM_LEDS, 0.6, 0.3, true, 8), 0);
        m_candle.animate(new FireAnimation(1.0, 0.1, NUM_LEDS, 0.6, 0.3, false, NUM_LEDS - STRIP_LENGTH), 1);
    }

    public void setActive() {
        m_currentMode = CANdleMode.ACTIVE;
        for (int i = 0; i < 8; i++) {
            m_candle.clearAnimation(i);
        }
    }

    public void setThrowOnGround(boolean state) {
        m_throwOnGround = state;
        if (state) {
            m_candle.animate(new StrobeAnimation(m_color.r, m_color.g, m_color.b, 0, 1e-5, NUM_LEDS), 0);
        }
    }

    public void setSnappedState(boolean state) {
        m_signal = state;
    }

    public boolean getSnappedState() {
        return m_signal;
    }

    public boolean getThrowOnGround() {
        return m_throwOnGround;
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
        SmartDashboard.putString("Mode: ", m_currentMode.name());
        if (m_currentMode == CANdleMode.ACTIVE) {
            if (!OneMechanism.getAutoAlignMode() && !m_throwOnGround && !m_signal && !m_fade) {
                m_candle.setLEDs(m_color.r, m_color.g, m_color.b);
            }
        } else if (m_currentMode == CANdleMode.FIRE) {
            if (!OneMechanism.getAutoAlignMode() && !m_throwOnGround && !m_signal && !m_fade) {
                m_candle.setLEDs(m_color.r, m_color.g, m_color.b, 0, 0, 8);
            }
        }
    }
}