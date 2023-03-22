// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.OneMechanism;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class LEDs extends SubsystemBase {
    private Color m_color, m_lastColor;
    private CANdle m_candle;
    private boolean m_blinking;

    private static LEDs m_instance;

    /**
     * the colors that the CANdle needs to be set to
     */
    public enum Color {
        GREEN(0, 254, 0), 
        PURPLE(118, 0, 254), 
        ORANGE(254, 55, 0), 
        WHITE(254, 254, 254),
        RED(254, 0, 0),
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
        m_candle = new CANdle(21, "rio");

        // If the brightness needs to go up, go NO HIGHER than 0.75
        m_candle.configBrightnessScalar(0.5); 
        m_candle.configLEDType(LEDStripType.GRB);
        m_candle.configLOSBehavior(true);

        setColor(Color.OFF);

        m_blinking = false;
    }

    /**
     * Sets the color of the LEDs.
     * @param color the color to set
     */
    public void setColor(Color color) {
        m_lastColor = m_color;
        m_color = color;
    }

    /**
     * sets the color to blank and runs {@code setLEDs()}
     */
    public void setBlank() {
        setColor(Color.OFF);
    }

    public void setClimbColor() {
        setColor(Color.GREEN);
    }

    /**
     * sets the color to the cone color (yellow)
     */
    public void setOrangeConeColor() {
        setColor(Color.ORANGE);
    }

    /**
     * sets the color to the cube color (purple)
     */
    public void setPurpleCubeColor() {
        setColor(Color.PURPLE);
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

    public SequentialCommandGroup blink(Color color) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_blinking = true),
            new InstantCommand(() -> setBlank()),
            new WaitCommand(0.02),
            new InstantCommand(() -> setColor(color)),
            new WaitCommand(0.02),
            new InstantCommand(() -> setBlank()),
            new WaitCommand(0.02),
            new InstantCommand(() -> setColor(color)),
            new InstantCommand(() -> m_blinking = false)
        );
    }

    public SequentialCommandGroup alternateBlink(Color color) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_blinking = true),
            new InstantCommand(() -> setColor(color)),
            new WaitCommand(0.02),
            new InstantCommand(() -> setColor(m_lastColor)),
            new WaitCommand(0.02),
            new InstantCommand(() -> setColor(color)),
            new WaitCommand(0.02),
            new InstantCommand(() -> setColor(m_lastColor)),
            new WaitCommand(0.02),
            new InstantCommand(() -> m_blinking = false)
        );
    }
    

    @Override
    public void periodic() {
        if (m_blinking == false) {
            if (OneMechanism.getClimbMode()) {
                setColor(Color.GREEN);
            }
            else if (OneMechanism.getAutoAlignMode()) {
                setColor(Color.RED);
            }
            else if (OneMechanism.getGamePieceMode() == OneMechanism.GamePieceMode.ORANGE_CONE) {
                setColor(Color.ORANGE);
            }
            else if (OneMechanism.getGamePieceMode() == OneMechanism.GamePieceMode.PURPLE_CUBE) {
                setColor(Color.PURPLE);
            }
            else {
                setBlank();
            }
        }
        // Set the physical CANdle LEDs to appropriate color.
        m_candle.setLEDs(m_color.r, m_color.g, m_color.b);
    }
}