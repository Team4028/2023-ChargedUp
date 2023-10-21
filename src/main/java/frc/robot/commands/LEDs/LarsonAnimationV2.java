// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;

public class LarsonAnimationV2 extends Command {
    private int direction, r, g, b, w, startIndex, upperBound, lowerBound, size, counter;
    private double speed;
    private V2BounceMode m_mode;
    private CANdle m_candle;
    private LEDs m_leds;
    private Timer m_timer;

    public enum V2BounceMode {
        FRONT, CENTRE, BACK;
    }

    // @formatter:off
    /**
     * Animates the CANdle with a custom made LarsonAnimation
     * 
     * @param r
     *            the red value [0, 254]
     * @param g
     *            the green value [0, 254]
     * @param b
     *            the blue value [0, 254]
     * @param w
     *            the white value [0, 254]
     * @param speed
     *            the speed of the animation, [0, 1]
     * @param inverted
     *            whether the led direction is inverted.
     *            <p>
     *            <pre>
     *{@code true:} downstream
     *{@code false:} upstream
     *
     * @param upperBoundIndex
     *            the numerically higher bound of the animation
     * @param lowerBoundIndex
     *            the numerically lower bound of the animation
     * @param startIndex
     *            the led index to start the animation at
     * @param size
     *            the amount of leds the animation controls at a time
     * @param mode
     *            this controls the way the animation bounces. See
     *            {@link com.ctre.phoenix.led.LarsonAnimation.BounceMode} for more
     *            information on how this works, or go 
     *            <a href="https://store.ctr-electronics.com/content/api/java/html/enumcom_1_1ctre_1_1phoenix_1_1led_1_1_larson_animation_1_1_bounce_mode.html">here</a>.
     * @param leds
     *            the led subsystem to run the animation on
     */
    // @formatter:on
    public LarsonAnimationV2(int r, int g, int b, int w, double speed, boolean inverted, int upperBoundIndex,
        int lowerBoundIndex, int startIndex, int size, V2BounceMode mode, LEDs leds) {
        m_timer = new Timer();
        m_timer.reset();
        m_timer.stop();
        // limits for the r g b and w values, (0-255)
        this.r = (r > 254 || r < 0) ? (r < 0 ? 0 : 254) : r;
        this.g = (g > 254 || g < 0) ? (g < 0 ? 0 : 254) : g;
        this.b = (b > 254 || b < 0) ? (b < 0 ? 0 : 254) : b;
        this.w = (w > 254 || w < 0) ? (w < 0 ? 0 : 254) : w;
        // limits the speed to a value from 0-1
        this.speed = (speed > 1 || speed <= 0) ? (speed > 1 ? 1 : 0) : speed;
        // limits the start index to be inside the bounds
        this.startIndex = (startIndex > upperBoundIndex || startIndex < lowerBoundIndex)
            ? (startIndex < lowerBoundIndex ? lowerBoundIndex : upperBoundIndex)
            : startIndex;
        this.direction = inverted ? -1 : 1;
        // checks if there are enough leds to supply the bounds
        this.upperBound = (upperBoundIndex > leds.getNumLEDs() || upperBoundIndex <= 0)
            ? (upperBoundIndex <= 0 ? 1 : leds.getNumLEDs())
            : upperBoundIndex;
        this.lowerBound = lowerBoundIndex < 0 ? 0 : (lowerBoundIndex > upperBoundIndex ? 0 : lowerBoundIndex);
        // checks that the size is within the bounds
        this.size = size > (upperBoundIndex - lowerBoundIndex) ? (upperBoundIndex - lowerBoundIndex) : size;
        m_mode = mode;
        m_leds = leds;
        m_candle = m_leds.getCandle();
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_timer.start();
        m_candle.setLEDs(0, 0, 0);
        counter = 0;
        // sets the original startpos of the led block
        m_candle.setLEDs(r, g, b, 0, startIndex, size);
    }

    @Override
    public void execute() {
        if (m_timer.get() >= (1 - speed) / 20) {
            switch (m_mode) {
                case FRONT:
                    if (((startIndex + counter + size) < upperBound || direction != 1)
                        && (direction != -1 || (counter + startIndex) > lowerBound)) {
                        // advance the led block
                        counter += direction;
                    } else {
                        // turn around
                        direction = -direction;
                    }
                    break;
                case CENTRE:
                    if (((startIndex + counter + (size / 2)) < upperBound || direction != -1)
                        && ((startIndex + counter + (size / 2)) > lowerBound || direction != 1)) {
                        // advance the led block
                        counter += direction;
                    } else {
                        // turn around
                        direction = -direction;
                    }
                    break;
                case BACK:
                    if (((counter + startIndex) < upperBound || direction != -1) && ((startIndex + counter + size) > lowerBound || direction != 1)) {
                        // advance the led block
                        counter += direction;
                    } else {
                        // turn around
                        direction = -direction;
                    }
                    break;
                default:
                    if ((startIndex + counter + size) < upperBound && (counter + startIndex) > lowerBound) {
                        // advance the led block
                        counter += direction;
                    } else {
                        // turn around
                        direction = -direction;
                    }
                    break;
            }

            // sets the previous led to the one set to 0, 0, 0, 0
            m_candle.setLEDs(0, 0, 0, 0,
                ((startIndex + counter) + (size * Integer.signum(-direction + 1))) - direction, 1);
            // updates the led block pos on the strip
            m_candle.setLEDs(r, g, b, w, startIndex + counter, size);
            m_timer.reset();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_timer.reset();
        m_timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
