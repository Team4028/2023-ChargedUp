// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.CANdleMode;

public class LarsonAnimationV2 extends CommandBase {
    private int direction, r, g, b, w, startIndex, upperBound, lowerBound, size, counter;
    private double speed;
    private V2BounceMode m_mode;
    private CANdle m_candle;
    private LEDs m_leds;

    public enum V2BounceMode {
        FRONT, CENTRE, BACK;
    }

    // @formatter:off
    /**
     * Animates the CANdle with a custom made LarsonAnimation
     * 
     * @param r
     *            the red value [0, 255]
     * @param g
     *            the green value [0, 255]
     * @param b
     *            the blue value [0, 255]
     * @param w
     *            the white value [0, 255]
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
     *            information on how this works.
     * @param leds
     *            the led subsystem to run the animation on
     */
    // @formatter:on
    public LarsonAnimationV2(int r, int g, int b, int w, double speed, boolean inverted, int upperBoundIndex,
        int lowerBoundIndex,
        int startIndex, int size, V2BounceMode mode, LEDs leds) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.r = (r > 255 || r < 0) ? (r < 0 ? 0 : 255) : r;
        this.g = (g > 255 || g < 0) ? (g < 0 ? 0 : 255) : g;
        this.b = (b > 255 || b < 0) ? (b < 0 ? 0 : 255) : b;
        this.w = (w > 255 || w < 0) ? (w < 0 ? 0 : 255) : w;
        this.speed = speed > 1 ? 1 : speed;
        this.startIndex = (startIndex > upperBoundIndex || startIndex < lowerBoundIndex)
            ? (startIndex < lowerBoundIndex ? lowerBoundIndex : upperBoundIndex)
            : startIndex;
        this.m_mode = mode;
        this.direction = inverted ? -1 : 1;
        this.upperBound = (upperBoundIndex > leds.getNumLEDs() || upperBoundIndex <= 0)
            ? (upperBoundIndex <= 0 ? 1 : leds.getNumLEDs())
            : upperBoundIndex;
        this.lowerBound = lowerBoundIndex < 0 ? 0 : (lowerBoundIndex > upperBoundIndex ? 0 : lowerBoundIndex);
        this.size = size > (upperBoundIndex - lowerBoundIndex) ? (upperBoundIndex - lowerBoundIndex) : size;
        m_leds = leds;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        counter = 0;
        m_candle.setLEDs(r, g, b, 0, startIndex, size);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        new WaitCommand((1 - speed) / 4).schedule();
        switch (m_mode) {
            case FRONT:
                if (counter + size < upperBound && counter > lowerBound) {
                    counter += Integer.signum(direction);
                } else {
                    direction = -direction;
                }
                break;
            case CENTRE:
                if (counter + (size / 2) < upperBound && counter + (size / 2) > lowerBound) {
                    counter += Integer.signum(direction);
                } else {
                    direction = -direction;
                }
                break;
            default:
                if (counter < upperBound && counter + size > lowerBound) {
                    counter += Integer.signum(direction);
                } else {
                    direction = -direction;
                }
                break;
        }
        m_candle.setLEDs(0, 0, 0, 0, (startIndex + counter) - Integer.signum(direction), 1);
        m_candle.setLEDs(r, g, b, w, startIndex + counter, size);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_leds.getMode() != CANdleMode.IDLEV2;
    }
}
