package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.lib.beaklib.drive.BeakDrivetrain;
import frc.lib.beaklib.units.Distance;
import frc.robot.commands.arm.RunArmsSafely;
import frc.robot.commands.auton.GeneratePathWithArc;
import frc.robot.commands.chassis.AddVisionMeasurement;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.LEDs.Color;
import frc.robot.subsystems.arms.LowerArm;
import frc.robot.subsystems.arms.UpperArm;
import frc.robot.subsystems.manipulator.Wrist;

/**
 * "This game looks like it'll be one mechanism and a controls game"
 * - Carson, correctly
 */
public class OneMechanism {
    
    private static final Distance FIELD_WIDTH = new Distance(8.0137);

    private static ScoringPositions currentPosition = ScoringPositions.STOWED;
    /**
     * the states of the robot
     */
    public enum GamePieceMode {
        ORANGE_CONE, 
        PURPLE_CUBE;
    }

    // @formatter:off
    public enum ScoringPositions {
        STOWED(                     5.00,       7.0,       305.0),
        INTERMEDIATE_LOW(           15.0,       13.0,       275.0),
        ACQUIRE_FLOOR_CUBE(         9.0,       23.0,       245.0),
        SCORE_MID_CUBE(             39.0,       6.00,       215.0), 
        SCORE_HIGH_CUBE(            51.0,       34.0,       203.0),
        ACQUIRE_FLOOR_CONE_TIPPED(  9.0,       26.5,       260.0),
        ACQUIRE_FLOOR_CONE_UPRIGHT( 8.50,       19.6,       262.0),
        SCORE_MID_CONE(             39.0,       6.00,       215.0), 
        SCORE_HIGH_CONE(            51.0,       34.0,       203.0),
        ACQUIRE_SINGLE_SUBSTATION(  2.60,       1.00,       320.0);

        public double lowerPosition;
        public double upperPosition;
        public double wristAngle;
        

        private ScoringPositions(double lowerPosition, double upperPosition, double wristAngle) {
            this.lowerPosition = lowerPosition;
            this.upperPosition = upperPosition;
            this.wristAngle = wristAngle;
        }
    }
    // @formatter:on

    /**
     * Represents a scoring node on the field.
     */
    static class Node {
        public Pose2d BluePose;
        public Pose2d RedPose;

        public int TagID;
        public int GridID;

        public GamePieceMode GamePiece;

        /**
         * Create a new Node.
         * 
         * @param gridID
         *            This should be equivalent to the array index.
         * @param tagID
         *            The tag ID of the node. Set to 0 if none.
         * @param gamePiece
         *            Whether this node stores a cone or a cube.
         * @param pose
         *            The BLUE pose of the node.
         */
        public Node(int gridID, int tagID, GamePieceMode gamePiece, Pose2d pose) {
            this.GridID = gridID;
            this.TagID = tagID;

            this.GamePiece = gamePiece;

            this.BluePose = pose;
            this.RedPose = new Pose2d(
                pose.getX(),
                FIELD_WIDTH.getAsMeters() - pose.getY(),
                pose.getRotation());
        }
    }

    public static final List<Node> NODES = Arrays.asList(
        // This list starts at the node nearest the opposing alliance's LOADING ZONE
        new Node(0, 0, GamePieceMode.ORANGE_CONE, new Pose2d(2.05, 4.94, new Rotation2d(Math.PI))), // 1
        new Node(1, 3, GamePieceMode.PURPLE_CUBE, new Pose2d(2.05, 4.45, new Rotation2d(Math.PI))), // tag
        new Node(2, 0, GamePieceMode.ORANGE_CONE, new Pose2d(2.05, 3.86, new Rotation2d(Math.PI))),
        new Node(3, 0, GamePieceMode.ORANGE_CONE, new Pose2d(2.05, 3.30, new Rotation2d(Math.PI))),
        new Node(4, 2, GamePieceMode.PURPLE_CUBE, new Pose2d(2.05, 2.75, new Rotation2d(Math.PI))),
        new Node(5, 0, GamePieceMode.ORANGE_CONE, new Pose2d(2.05, 2.21, new Rotation2d(Math.PI))),
        new Node(6, 0, GamePieceMode.ORANGE_CONE, new Pose2d(2.05, 1.63, new Rotation2d(Math.PI))),
        new Node(7, 1, GamePieceMode.PURPLE_CUBE, new Pose2d(2.05, 1.08, new Rotation2d(Math.PI))),
        new Node(8, 0, GamePieceMode.ORANGE_CONE, new Pose2d(2.05, 0.42, new Rotation2d(Math.PI))) // 9
    );

    private static Node m_currentNode = NODES.get(0);

    private static GamePieceMode m_currentMode = GamePieceMode.ORANGE_CONE;

    private static LEDs m_leds;
    private static BeakDrivetrain m_drive;
    private static Vision m_vision;
    private static UpperArm m_upperArm;
    private static LowerArm m_lowerArm;
    private static Wrist m_wrist;

    private static boolean m_climbMode = false;
    private static boolean m_autoAlignMode = false;
    private static boolean m_areTheLightsOn = false;

    /**
     * Turns of the CANdle
     */
    public static void killTheLights() {
        m_areTheLightsOn = false;
        if (!m_climbMode) {
            m_leds.setBlank();
        }
    }

    /**
     * sets the robot mode to Orange (cone) mode
     */
    public static void becomeOrangeMode() {
        m_areTheLightsOn = true;
        m_currentMode = GamePieceMode.ORANGE_CONE;
        if (!m_climbMode) {
            m_leds.setOrangeConeColor();
            m_leds.blink();
        }
    }

    /**
     * sets the robot mode to Purple (cube) mode 
     */
    public static void becomePurpleMode() {
        m_areTheLightsOn = true;
        m_currentMode = GamePieceMode.PURPLE_CUBE;
        if (!m_climbMode) {
            m_leds.setPurpleCubeColor();
            m_leds.blink();
        }
    }

    public static void toggleAutoAlign() {
        m_autoAlignMode = !m_autoAlignMode;
        SmartDashboard.putBoolean("Auto Align", m_autoAlignMode);
    }

    public static void toggleGreen() {
        m_climbMode = !m_climbMode;
        if (m_climbMode) {
            m_leds.setClimbColor();
        } else {
            switch (getGamePieceMode()) {
                case ORANGE_CONE:
                    m_leds.setOrangeConeColor();
                    break;
                case PURPLE_CUBE:
                    m_leds.setPurpleCubeColor();
                    break;
                default:
                    m_leds.setOrangeConeColor();
                    break;
            }
        }
        if (m_areTheLightsOn == true) {
            m_leds.blink();
        }
    }

    /**
     * switches between cone and cube mode
     */
    public static void toggleGamePieceMode() {
        switch (m_currentMode) {
            case ORANGE_CONE:
                m_currentMode = GamePieceMode.PURPLE_CUBE;
                break;
            case PURPLE_CUBE:
                m_currentMode = GamePieceMode.ORANGE_CONE;
                break;
            default:
                m_currentMode = GamePieceMode.ORANGE_CONE;
                break;
        }
    }

    public static Node getNodeFromTagID(int id) {
        if (id == 0) {
            return m_currentNode;
        }
        for (Node node : NODES) {
            if (node.TagID == id || 9 - node.TagID == id) {
                return node;
            }
        }

        return NODES.get(0);
    }

    public static Command setNode(Supplier<Node> node) {
        return new InstantCommand(() -> {
            m_currentNode = node.get();
            SmartDashboard.putNumber("Node", m_currentNode.GridID);
        });
    }

    public static Command runToNodePosition() {
        // Add measurements to the pose estimator before and after to ensure relative
        // accuracy
        return new ConditionalCommand(
            // new AddVisionMeasurement(m_drive, m_vision).andThen(
            new GeneratePathWithArc(
                () -> DriverStation.getAlliance() == Alliance.Red ? m_currentNode.RedPose : m_currentNode.BluePose,
                m_drive).deadlineWith(
                    new RepeatCommand(new AddVisionMeasurement(m_drive,
                        m_vision)))
                    .withInterruptBehavior(InterruptionBehavior.kCancelSelf),
            // .andThen(new AddVisionMeasurement(m_drive, m_vision))),
            Commands.none(),
            () -> m_autoAlignMode);
    }

    public static Command setNodeFromTagID(Supplier<Integer> id) {
        return setNode(() -> getNodeFromTagID(id.get())).andThen(runToNodePosition());
    }

    public static Node getCurrentNode() {
        return m_currentNode;
    }

    /**
     * Increments the node index (moves FROM the side closest to the loading zone TO
     * the other side)
     * 
     * @return A {@link Command} to increment the node.
     */
    public static Command incrementNode() {
        return new InstantCommand(
            () -> {
                // weird solution but it works
                List<Node> nodeToSet = new ArrayList<Node>();
                nodeToSet.add(m_currentNode);

                // Searches for the next node with the same state.
                for (Node node : NODES) {
                    if (node.GridID > m_currentNode.GridID && node.GamePiece == m_currentMode) {
                        nodeToSet.set(0, node);
                        break;
                    }
                }

                setNode(() -> nodeToSet.get(0)).schedule();
            }).andThen(runToNodePosition());
    }

    /**
     * Decrements the node index (moves FROM the side closest to the cable cover TO
     * the other side)
     * 
     * @return A {@link Command} to increment the node.
     */
    public static Command decrementNode() {
        return new InstantCommand(
            () -> {
                // weird solution but it works
                List<Node> nodeToSet = new ArrayList<Node>();
                nodeToSet.add(m_currentNode);

                // Searches for the next node (BACKWARDS!) with the same state.
                for (int i = NODES.size() - 1; i > -1; i--) {
                    Node node = NODES.get(i);
                    if (node.GridID < m_currentNode.GridID && node.GamePiece == m_currentMode) {
                        nodeToSet.set(0, node);
                        break;
                    }
                }

                setNode(() -> nodeToSet.get(0)).schedule();
            }).andThen(runToNodePosition());
    }

    public static void addSubsystems(LEDs leds, BeakDrivetrain drivetrain, Vision vision, LowerArm lowerArm, UpperArm upperArm, Wrist wrist) {
        m_leds = leds;
        m_drive = drivetrain;
        m_vision = vision;
        m_upperArm = upperArm;
        m_lowerArm = lowerArm;
        m_wrist = wrist;
    }
    
    public static void setScoringPosition(ScoringPositions pos) {
        currentPosition = pos;
    }

    public static ScoringPositions getScoringPosition() {
        return currentPosition;
    }
    
    /**
     * 
     * @return the state of the robot
     */
    public static GamePieceMode getGamePieceMode() {
        return m_currentMode;
    }

    /**
     * 
     * @return whether or not the robot is in climb mode
     */
    public static boolean getClimbMode() {
        return m_climbMode;
    }

    public static boolean getAutoAlignMode() {
        return m_autoAlignMode;
    }

    public static boolean getLightMode() {
        return m_areTheLightsOn;
    }

    public static Command runArms(ScoringPositions targetPos) {
        return new RunArmsSafely(targetPos, m_lowerArm, m_upperArm, m_wrist);
    }

    public static void signalAcquisition() {
        m_leds.alternateBlink(Color.WHITE);
    }
}