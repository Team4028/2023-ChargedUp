package frc.robot;

import frc.robot.commands.BlinkLEDs;
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
        if(!climbMode){
            m_leds.setBlank();
        }
    }

    public static void modeCone() {
        m_currentState = State.CONE;
        if(!climbMode){
            m_leds.setCone();
            new BlinkLEDs(m_leds).schedule();
        }
    }

    public static void modeCube() {
        m_currentState = State.CUBE;
        if(!climbMode){
            m_leds.setCube();
            new BlinkLEDs(m_leds).schedule();
        }
    }
    
    public static void toggleClimb() {
        climbMode = !climbMode;
        if(climbMode){
            m_leds.setClimb();
        } else{
            switch(getState()){
                case OFF:
                    m_leds.setBlank();
                    break;
                case CONE:
                    m_leds.setCone();
                    break;
                case CUBE:
                    m_leds.setCube();
                    break;
                default:
                    m_leds.setCone();
                    break;
            }
        }
        if(getState()!=State.OFF){
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
