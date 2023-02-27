package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.beaklib.drive.BeakDrivetrain;
import frc.lib.beaklib.units.Distance;
import frc.robot.commands.BlinkLEDs;
import frc.robot.commands.auton.GeneratePathWithArc;
import frc.robot.subsystems.LEDs;

public class RobotState {
    private static final Distance FIELD_WIDTH = new Distance(8.0137);

    public enum State {
        CONE, OFF, CUBE;
    }

    /**
     * Represents a scoring node on the field.
     */
    static class Node {
        public Pose2d BluePose;
        public Pose2d RedPose;

        public int TagID;
        public int GridID;

        /**
         * Create a new Node.
         * 
         * @param tagID
         *            The tag ID of the node. Set to 0 if none.
         * @param pose
         *            The BLUE pose of the node.
         */
        public Node(int gridID, int tagID, Pose2d pose) {
            this.GridID = gridID;
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
        new Node(0, 0, new Pose2d(2.0, 4.94, new Rotation2d(Math.PI))), // 1
        new Node(1, 3, new Pose2d(2.0, 4.45, new Rotation2d(Math.PI))), // tag
        new Node(2, 0, new Pose2d(2.0, 3.86, new Rotation2d(Math.PI))),
        new Node(3, 0, new Pose2d(2.0, 3.30, new Rotation2d(Math.PI))),
        new Node(4, 2, new Pose2d(2.0, 2.75, new Rotation2d(Math.PI))),
        new Node(5, 0, new Pose2d(2.0, 2.21, new Rotation2d(Math.PI))),
        new Node(6, 0, new Pose2d(2.0, 1.63, new Rotation2d(Math.PI))),
        new Node(7, 1, new Pose2d(2.0, 1.08, new Rotation2d(Math.PI))),
        new Node(8, 0, new Pose2d(2.0, 0.50, new Rotation2d(Math.PI))) // 9
    );

    private static Node m_currentNode = NODES.get(0);

    private static State m_currentState = State.CONE;

    private static LEDs m_leds;
    private static BeakDrivetrain m_drive;

    private static boolean climbMode = false;
    private static boolean autoAlignMode = false;

    private static Field2d field = new Field2d();

    public static void modeBlank() {
        m_currentState = State.OFF;
        if (!climbMode) {
            m_leds.setBlank();
        }
    }

    public static void modeCone() {
        m_currentState = State.CONE;
        if (!climbMode) {
            m_leds.setCone();
            new BlinkLEDs(m_leds).schedule();
        }
    }

    public static void modeCube() {
        m_currentState = State.CUBE;
        if (!climbMode) {
            m_leds.setCube();
            new BlinkLEDs(m_leds).schedule();
        }
    }

    public static void toggleAutoAlign() {
        SmartDashboard.putBoolean("Auto Align", autoAlignMode);
        autoAlignMode = !autoAlignMode;
    }

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
        if (getState() != State.OFF) {
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

    public static Command setNode(Node node) {
        return new InstantCommand(() -> m_currentNode = node);
    }

    public static Command runToNodePosition() {
        SmartDashboard.putNumber("Node", m_currentNode.GridID);

        if (autoAlignMode) {
            return new GeneratePathWithArc(
                () -> DriverStation.getAlliance() == Alliance.Red ? m_currentNode.RedPose : m_currentNode.BluePose,
                m_drive);
        } else {
            return Commands.none();
        }
    }

    public static Command setNodeFromTagID(int id) {
        return setNode(getNodeFromTagID(id)).andThen(runToNodePosition());
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
                Node node;
                if (m_currentNode.GridID + 1 == NODES.size()) {
                    node = m_currentNode;
                } else {
                    node = NODES.get(m_currentNode.GridID + 1);
                }

                setNode(node).schedule();
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
                Node node;
                if (m_currentNode.GridID - 1 < 0) {
                    node = m_currentNode;
                } else {
                    node = NODES.get(m_currentNode.GridID - 1);
                }
                
                setNode(node).schedule();
            }).andThen(runToNodePosition());
    }

    public static void addSubsystem(LEDs leds, BeakDrivetrain drivetrain) {
        m_leds = leds;
        m_drive = drivetrain;
    }
}