package frc.robot;

import java.util.Arrays;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.beaklib.units.Distance;
import frc.robot.commands.BlinkLEDs;
import frc.robot.subsystems.LEDs;

public class RobotState {
    
    private static final Distance FIELD_WIDTH = new Distance(8.0137);

    private static ScoringPositions currentPosition = ScoringPositions.STOWED;
    /**
     * the states of the robot
     */
    public enum GamePieceMode {
        CONE, OFF, CUBE;
    }

    public enum ScoringPositions {
        STOWED(1., 1., 320.0),
        INTERMEDIATE_LOW(3., 6.,286.0),
        SCORE_MID(13., 0.5, 190.0), 
        SCORE_HIGH(13., 19.0, 255.0),
        ACQUIRE_FLOOR_CUBE(2.0, 15.0, 245.0),
        ACQUIRE_FLOOR_TIPPED_CONE(2.0, 15.0, 245.0),
        ACQUIRE_FLOOR_UPRIGHT_CONE(2.0, 4.5, 241.);

        public double lowerPosition;
        public double upperPosition;
        public double wristAngle;

        private ScoringPositions(double lowerPosition, double upperPosition, double wristAngle) {
            this.lowerPosition = lowerPosition;
            this.upperPosition = upperPosition;
            this.wristAngle=wristAngle;
        }
    }

    /**
     * Represents a scoring node on the field.
     */
    static class Node {
        public Pose2d BluePose;
        public Pose2d RedPose;

        public int TagID;

        /**
         * Create a new Node.
         * 
         * @param tagID
         *            The tag ID of the node. Set to 0 if none.
         * @param pose
         *            The BLUE pose of the node.
         */
        public Node(int tagID, Pose2d pose) {
            this.TagID = tagID;
            this.BluePose = pose;
            this.RedPose = new Pose2d(
                pose.getX(),
                FIELD_WIDTH.getAsMeters() - pose.getY(),
                pose.getRotation());
        }
    }

    public static final List<Node> NODES = Arrays.asList(
        // This list starts at the node nearest the opposing alliance's LOADING ZONE
        new Node(0, new Pose2d(1.80, 4.94, new Rotation2d(Math.PI))), // 1
        new Node(3, new Pose2d(1.80, 4.45, new Rotation2d(Math.PI))), // tag
        new Node(0, new Pose2d(1.80, 3.86, new Rotation2d(Math.PI))),
        new Node(0, new Pose2d(1.80, 3.30, new Rotation2d(Math.PI))),
        new Node(2, new Pose2d(1.80, 2.75, new Rotation2d(Math.PI))),
        new Node(0, new Pose2d(1.80, 2.21, new Rotation2d(Math.PI))),
        new Node(0, new Pose2d(1.80, 1.63, new Rotation2d(Math.PI))),
        new Node(1, new Pose2d(1.80, 1.08, new Rotation2d(Math.PI))),
        new Node(0, new Pose2d(1.80, 0.50, new Rotation2d(Math.PI))) // 9
    );

    private static Node m_currentNode = NODES.get(0);

    private static GamePieceMode m_currentMode = GamePieceMode.CONE;

    private static LEDs m_leds;

    private static boolean climbMode = false;

    /**
     * Turns of the CANdle
     */
    public static void modeBlank() {
        m_currentMode = GamePieceMode.OFF;
        if (!climbMode) {
            m_leds.setBlank();
        }
    }

    /**
     * sets the robot mode to cone mode
     */
    public static void modeCone() {
        m_currentMode = GamePieceMode.CONE;
        if (!climbMode) {
            m_leds.setCone();
            new BlinkLEDs(m_leds).schedule();
        }
    }

    /**
     * sets the robot mode to cube mode 
     */
    public static void modeCube() {
        m_currentMode = GamePieceMode.CUBE;
        if (!climbMode) {
            m_leds.setCube();
            new BlinkLEDs(m_leds).schedule();
        }
    }

    /**
     * toggles the robot mode between climb mode
     */
    public static void toggleClimb() {
        climbMode = !climbMode;
        if (climbMode) {
            m_leds.setClimb();
        } else {
            switch (getState()) {
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
        if (getState() != GamePieceMode.OFF) {
            new BlinkLEDs(m_leds).schedule();
        }
    }

    /**
     * switches between cone and cube mode
     */
    public static void toggle() {
        switch (m_currentMode) {
            case CONE:
                m_currentMode = GamePieceMode.CUBE;
                break;
            case CUBE:
                m_currentMode = GamePieceMode.CONE;
                break;
            default:
                m_currentMode = GamePieceMode.CONE;
                break;
        }
    }

    /**
     * 
     * @return the state of the robot
     */
    public static GamePieceMode getState() {
        return m_currentMode;
    }

    /**
     * 
     * @return whether or not the robot is in climb mode
     */
    public static boolean getClimb() {
        return climbMode;
    }

    public static Node getNodeFromTagID(int id) {
        for (Node node : NODES) {
            if (node.TagID == id || 9 - node.TagID == id) {
                return node;
            }
        }

        return new Node(0, new Pose2d());
    }

    public static void setNode(Node node) {
        m_currentNode = node;
    }

    public static Node getCurrentNode() {
        return m_currentNode;
    }

    public static void addSubsystem(LEDs leds) {
        m_leds = leds;
    }
    
    public static void setScoringPosition(ScoringPositions pos){
        currentPosition = pos;
    }

    public static ScoringPositions geScoringPosition(){
        return currentPosition;
    }
}
