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
        GREEN(0, 255, 0), PURPLE(127, 0, 255), YELLOW(255, 255, 0), OFF(0, 0, 0);

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

        m_candle.configBrightnessScalar(1.0);
        m_candle.configLEDType(LEDStripType.GRB);
        m_candle.configLOSBehavior(true);

        setColor(Color.OFF);

        m_blinking = false;
    }

    /**
     * sets the color of the LED class
     * @param color the color to set
     */
    public void setColor(Color color) {
        m_color = color;
    }

    /**
     * sets the color to blank and runs {@code setLEDs()}
     */
    public void setBlank() {
        m_lastColor = m_color;
        setColor(Color.OFF);
    }

    public void setClimb() {
        setColor(Color.GREEN);
    }

    /**
     * sets the color to the cone color (yellow)
     */
    public void setCone() {
        setColor(Color.YELLOW);
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

    
    public SequentialCommandGroup blink() {
        return new SequentialCommandGroup(
            new WaitCommand(0.1),
            new InstantCommand(() -> m_blinking = true),
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
            new InstantCommand(() -> setColor(m_lastColor)),
            new WaitCommand(0.08),
            new InstantCommand(() -> setBlank()),
            new WaitCommand(0.08),
            new InstantCommand(() -> setColor(m_lastColor)),
            new InstantCommand(() -> m_blinking = false)
        );
    }

    public SequentialCommandGroup blink(Color color) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_blinking = true),
            new InstantCommand(() -> setColor(color)),
            new WaitCommand(0.1),
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
            new InstantCommand(() -> setColor(color)),
            new WaitCommand(0.08),
            new InstantCommand(() -> setBlank()),
            new WaitCommand(0.08),
            new InstantCommand(() -> setColor(color)),
            new InstantCommand(() -> m_blinking = false)
        );
    }

    public SequentialCommandGroup alternateBlink(Color color) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_blinking = true),
            new InstantCommand(() -> setColor(color)),
            new WaitCommand(0.1),
            new InstantCommand(() -> setColor(m_color)),
            new WaitCommand(0.08),
            new InstantCommand(() -> setColor(color)),
            new WaitCommand(0.08),
            new InstantCommand(() -> setColor(m_color)),
            new WaitCommand(0.08),
            new InstantCommand(() -> setColor(color)),
            new WaitCommand(0.08),
            new InstantCommand(() -> setColor(m_color)),
            new WaitCommand(0.08),
            new InstantCommand(() -> setColor(color)),
            new WaitCommand(0.08),
            new InstantCommand(() -> setColor(m_color)),
            new WaitCommand(0.08),
            new InstantCommand(() -> setColor(color)),
            new InstantCommand(() -> m_blinking = false)
        );
    }
    

    @Override
    public void periodic() {
        if (m_blinking == false) {
            if (OneMechanism.getClimb()) {
                setColor(Color.GREEN);
            }
            else if (OneMechanism.getState() == OneMechanism.GamePieceMode.CONE) {
                setColor(Color.YELLOW);
            }
            else if (OneMechanism.getState() == OneMechanism.GamePieceMode.CUBE) {
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