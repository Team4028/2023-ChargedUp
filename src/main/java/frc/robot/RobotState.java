package frc.robot;

import frc.robot.commands.BlinkLEDs;
import frc.robot.subsystems.LEDs;

public class RobotState {
    private static LEDs m_leds;
    public enum State {
        CONE,
        CUBE;
    }

    private static State m_currentState = State.CONE;
    private static boolean climbMode = false;

    public static void modeCone() {
        m_currentState = State.CONE;
        m_leds.setCone();
        new BlinkLEDs(m_leds).schedule();
    }

    public static void modeCube() {
        m_currentState = State.CUBE;
        m_leds.setCube();
        new BlinkLEDs(m_leds).schedule();
    }

    public static void toggleClimb() {
        climbMode = !climbMode;
        if(climbMode){
            m_leds.setClimb();
            new BlinkLEDs(m_leds).schedule();
        }
    }

    public static void toggle() {
        switch (m_currentState) {
            case CONE:
                m_currentState = State.CUBE;
                break;
            case CUBE:
                m_currentState = State.CONE;
                break;
            default:
                m_currentState = State.CONE;
                break;
        }
    }

    public static State getState() {
        return m_currentState;
    }

    public static boolean getClimb() {
        return climbMode;
    }

    public static void addSubsystem(LEDs leds){
        m_leds = leds;
    }
}
