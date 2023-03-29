// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
        ACTIVE, //
        IDLE, //
        SLIDE;
    }

    private CANdleMode m_currentMode;
    private Color m_color, m_lastColor;

    // We need to be able to update animations on-the-fly for color changes.
    private final List<Supplier<Animation>> m_currentAnimations;

    private final int NUM_LEDS = 119;
    private final int STRIP_LENGTH = 51;

    private CANdle m_candle;
    private boolean m_beacon = false;
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
        m_currentAnimations = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            m_currentAnimations.add(() -> null);
        }

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

    public SequentialCommandGroup blinkWhite() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                switch (OneMechanism.getGamePieceMode()) {
                    case PURPLE_CUBE:
                        m_lastColor = Color.PURPLE;
                        break;
                    case ORANGE_CONE:
                        m_lastColor = Color.ORANGE;
                        break;
                    default:
                        m_lastColor = Color.ORANGE;
                        break;
                }
            }),
            new InstantCommand(() -> setBlank()),
            new WaitCommand(0.02),
            new InstantCommand(() -> setColor(Color.WHITE)),
            new WaitCommand(0.02),
            new InstantCommand(() -> setBlank()),
            new WaitCommand(0.02),
            new InstantCommand(() -> setColor(Color.WHITE)),
            new WaitCommand(0.02),
            new InstantCommand(() -> setColor(m_lastColor)));
    }

    public SequentialCommandGroup blinkBeaconWhiteAndRed() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> setBeaconColor(Color.WHITE)),
            new WaitCommand(0.02),
            new InstantCommand(() -> setBeaconColor(Color.RED)),
            new WaitCommand(0.02),
            new InstantCommand(() -> setBeaconColor(Color.WHITE)),
            new WaitCommand(0.02),
            new InstantCommand(() -> setBeaconColor(Color.RED)),
            new WaitCommand(0.02),
            new InstantCommand(() -> setBeaconColor(Color.WHITE)),
            new WaitCommand(0.02),
            new InstantCommand(() -> setBeaconColor(Color.RED)));
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

    public void setBeaconColor(Color color) {
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
    public void setFade(boolean fade) {
        clearAnimations();
        m_fade = fade;
        if (fade) {
            // Create two separate animations if we have an active beacon.
            // There are different necessary parameters for beacon mode to avoid blinking
            // the beacon.
            m_currentAnimations.set(0, () -> new SingleFadeAnimation(m_color.r, m_color.g, m_color.b, 0, 0.8,
                OneMechanism.getBeaconState() ? STRIP_LENGTH : NUM_LEDS, OneMechanism.getBeaconState() ? 16 : 0));

            m_currentAnimations.set(1,
                () -> OneMechanism.getBeaconState() ? new SingleFadeAnimation(m_color.r, m_color.g, m_color.b, 0, 0.8,
                    STRIP_LENGTH + 1, STRIP_LENGTH + 8) : null);
        }

    }

    public boolean getFade() {
        return m_fade;
    }

    // TODO: Test \/
    public Command setIdle() {
        m_fade = false;
        m_currentMode = CANdleMode.IDLE;
        clearAnimations();
        return new ParallelCommandGroup(
            // Left
            new LarsonAnimationV2(Color.PURPLE.r, Color.PURPLE.g, Color.PURPLE.b, 0, 0.8, false, 59, 8, 8, 8,
                V2BounceMode.FRONT, this),
            new LarsonAnimationV2(Color.ORANGE.r, Color.ORANGE.g, Color.ORANGE.b, 0, 0.8, false, 59, 8, 16, 8,
                V2BounceMode.FRONT, this),
            new LarsonAnimationV2(Color.PURPLE.r, Color.PURPLE.g, Color.PURPLE.b, 0, 0.8, false, 59, 8, 24, 8,
                V2BounceMode.FRONT, this),
            new LarsonAnimationV2(Color.ORANGE.r, Color.ORANGE.g, Color.ORANGE.b, 0, 0.8, false, 59, 8, 32, 8,
                V2BounceMode.FRONT, this),
            // Right
            new LarsonAnimationV2(Color.PURPLE.r, Color.PURPLE.g, Color.PURPLE.b, 0, 0.8, true, NUM_LEDS, 68,
                NUM_LEDS - 8, 8,
                V2BounceMode.FRONT, this),
            new LarsonAnimationV2(Color.ORANGE.r, Color.ORANGE.g, Color.ORANGE.b, 0, 0.8, true, NUM_LEDS, 68,
                NUM_LEDS - 16, 8,
                V2BounceMode.FRONT, this),
            new LarsonAnimationV2(Color.PURPLE.r, Color.PURPLE.g, Color.PURPLE.b, 0, 0.8, true, NUM_LEDS, 68,
                NUM_LEDS - 24, 8,
                V2BounceMode.FRONT, this),
            new LarsonAnimationV2(Color.ORANGE.r, Color.ORANGE.g, Color.ORANGE.b, 0, 0.8, true, NUM_LEDS, 68,
                NUM_LEDS - 32, 8,
                V2BounceMode.FRONT, this));
    }

    public void setSlide() {
        m_fade = false;
        m_currentMode = CANdleMode.SLIDE;
        clearAnimations();

        // Two separate slides (one for each side)
        m_currentAnimations.set(0, () -> new ColorFlowAnimation(m_color.r, m_color.g, m_color.b, 0, 0.88, NUM_LEDS,
            Direction.Forward, OneMechanism.getBeaconState() ? 16 : 8));

        m_currentAnimations.set(1,
            () -> new ColorFlowAnimation(m_color.r, m_color.g, m_color.b, 0, 0.88, NUM_LEDS, Direction.Backward,
                0));
    }

    public void clearAnimations() {
        // Make each animation a blank (cleared) animation.
        for (int i = 0; i < m_currentAnimations.size(); i++) {
            m_currentAnimations.set(i, () -> null);
        }
    }

    public void setVictorySpin() {
        m_fade = false;
        m_currentMode = CANdleMode.VICTORY_SPIN;
        clearAnimations();

        m_currentAnimations.set(0, () -> new RainbowAnimation(1, 1, NUM_LEDS));
    }

    public void setFireWorkPlz() {
        m_fade = false;
        m_currentMode = CANdleMode.FIRE;
        clearAnimations();
        
        // Fire animations for each side.
        m_currentAnimations.set(0, () -> new FireAnimation(1.0, 0.2, STRIP_LENGTH, 0.4, 0.3, true, 8));
        m_currentAnimations.set(1,
            () -> new FireAnimation(1.0, 0.2, STRIP_LENGTH, 0.4, 0.3, false, NUM_LEDS - STRIP_LENGTH));
    }

    // TODO: ALL of this needs javadoc.
    public void setActive() {
        m_fade = false;
        m_currentMode = CANdleMode.ACTIVE;
        clearAnimations();
    }

    public void setBeaconState(boolean state) {
        m_beacon = state;
    }

    public boolean getBeaconState() {
        return m_beacon;
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
        SmartDashboard.putBoolean("animations 0", m_currentAnimations.get(0).get() != null);
        SmartDashboard.putBoolean("animations 1", m_currentAnimations.get(1).get() != null);
        SmartDashboard.putString("Mode", m_currentMode.name());

        // TODO: document this.
        if (m_currentMode == CANdleMode.ACTIVE) {
            if (!OneMechanism.getScoreMode() && !m_beacon && !m_fade) {
                m_candle.setLEDs(m_color.r, m_color.g, m_color.b);
            } else if (!OneMechanism.getScoreMode() && !m_fade) {
                m_candle.setLEDs(m_color.r, m_color.g, m_color.b, 0, 0, 8);
                m_candle.setLEDs(m_color.r, m_color.g, m_color.b, 0, 16, STRIP_LENGTH - 8);
                m_candle.setLEDs(m_color.r, m_color.g, m_color.b, 0, NUM_LEDS - STRIP_LENGTH, STRIP_LENGTH - 8);
            }
        } else if (m_currentMode == CANdleMode.FIRE) {
            if (!OneMechanism.getScoreMode() && !m_beacon && !m_fade) {
                m_candle.setLEDs(m_color.r, m_color.g, m_color.b, 0, 0, 8);
            }
        }

        OneMechanism.checkAuxiliaryModesPeriodic();
        for (int i = 0; i < m_currentAnimations.size(); i++) {
            m_candle.animate(m_currentAnimations.get(i).get(), i);
        }
    }
}