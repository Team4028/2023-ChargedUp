package frc.robot;

import frc.robot.subsystems.LEDs;
public class RobotState {
    private static LEDs m_leds;
    public enum State {
        CONE,
        OFF,
        CUBE;
    }

    private static State m_currentState = State.CONE;
    private static boolean climbMode = false;

    public static void modeBlank(){
        m_currentState = State.OFF;
    }

    public static void setConeMode() {
        m_currentState = State.CONE;
        m_leds.blink().schedule();
    }

    public static void setCubeMode() {
        m_currentState = State.CUBE;
        m_leds.blink().schedule();
    }
    
    public static void toggleClimb() {
        climbMode = !climbMode;
        m_leds.blink().schedule();
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
