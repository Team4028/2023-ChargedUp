// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private Color m_color;
    private CANdle m_candle;
    private int r, g, b;
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
    }

    /**
     * sets the leds to the color entered in {@link frc.robot.subsystems.LEDs}'s {@code setColor()}
     * @return a command that does the above task
     */
    public Command setLEDs() {
        return runOnce(() -> {
            this.r = m_color.r;
            this.g = m_color.g;
            this.b = m_color.b;
        });
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
        setColor(Color.OFF);
        setLEDs().ignoringDisable(true).schedule();
    }

    /**
     * sets the LEDs r, g, and b fields to 0
     * @return a command that does the above task
     */
    public Command setOff() {
        return runOnce(() -> {
            r = 0;
            g = 0;
            b = 0;
        });
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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("r", r);
        SmartDashboard.putNumber("g", g);
        SmartDashboard.putNumber("b", b);
        // setLEDs().schedule();
        m_candle.setLEDs(r, g, b);
    }
}