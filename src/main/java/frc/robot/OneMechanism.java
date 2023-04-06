package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.lib.beaklib.drive.swerve.BeakSwerveDrivetrain;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.arm.RunArmsWithPID;
import frc.robot.commands.auton.GeneratePathWithArc;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.LEDs.CANdleMode;
import frc.robot.subsystems.LEDs.Color;
import frc.robot.subsystems.arms.LowerArm;
import frc.robot.subsystems.arms.UpperArm;
import frc.robot.subsystems.manipulator.Wrist;

/**
 * "This game looks like it'll be one mechanism and a controls game"
 * - Carson, correctly
 */
public class OneMechanism {
    private static ScoringPositions currentPosition = ScoringPositions.STOWED;

    /**
     * the states of the robot
     */
    public enum GamePieceMode {
        ORANGE_CONE, PURPLE_CUBE;
    }

    // @formatter:off
    /**
     * <p>
     * <table>
     * <tr>
     *                           <th>Position:</th>                      <th>lowerPos:</th>             <th>upperPos:</th>          <th>wristAngle:</th>
     * </tr>
     * <tr>
     * <td>         ------------------------------------------</td> <td>--------------------</td> <td>--------------------</td> <td>--------------------</td>
     * </tr>
     * <tr>
     * <td>         STOWED:                                   </td> <td>3.0                 </td> <td>4.0                 </td> <td>305.0               </td>
     * </tr>
     * <tr>
     * <td>         INTERMEDIATE_LOW:                         </td> <td>9.5                 </td> <td>21.5                </td> <td>275.0               </td>
     * </tr>
     * <tr>
     * <td>         =========================                 </td> <td>============        </td> <td>============         </td> <td>============       </td>
     * </tr>
     * <tr>
     * <td>         SCORE_LOW_CUBE:                           </td> <td>15.0                </td> <td>13.0                 </td> <td>200.0              </td>
     * </tr>
     * <tr>
     * <td>         SCORE_MID_CUBE:                           </td> <td>39.0                </td> <td>6.0                  </td> <td>215.0              </td>
     * </tr>
     * <tr>
     * <td>         SCORE_HIGH_CUBE:                          </td> <td>51.0                </td> <td>34.0                 </td> <td>203.0              </td>
     * </tr>
     * <tr>
     * <td>         =========================                 </td> <td>============        </td> <td>============         </td> <td>============       </td>
     * </tr>
     * <tr>
     * <td>         AUTON_PREP_CUBE:                          </td> <td>51.0                </td> <td>4.0                  </td> <td>203.0              </td>
     * </tr>
     * <tr>
     * <td>         =========================                 </td> <td>============        </td> <td>============         </td> <td>============       </td>
     * </tr>
     * <tr>
     * <td>         ACQUIRE_FLOOR_CUBE:                       </td> <td>9.0                 </td> <td>23.0                 </td> <td>245.0              </td>
     * </tr>
     * <tr>
     * <td>         ACQUIRE_FLOOR_CONE_TIPPED:                </td> <td>9.0                 </td> <td>26.5                 </td> <td>260.0              </td>
     * </tr>
     * <tr>
     * <td>         ACQUIRE_FLOOR_CONE_UPRIGHT:               </td> <td>9.0                 </td> <td>19.6                 </td> <td>262.5              </td>
     * </tr>
     * <tr>
     * <td>         =========================                 </td> <td>============        </td> <td>============         </td> <td>============       </td>
     * </tr>
     * <tr>
     * <td>         SCORE_LOW_CONE:                           </td> <td>15.0                </td> <td>13.0                 </td> <td>200.0              </td>
     * </tr>
     * <tr>
     * <td>         SCORE_MID_CONE:                           </td> <td>39.0                </td> <td>6.0                  </td> <td>215.0              </td>
     * </tr>
     * <tr>
     * <td>         SCORE_HIGH_CONE:                          </td> <td>51.0                </td> <td>34.0                 </td> <td>203.0              </td>
     * </tr>
     * <tr>
     * <td>         =========================                 </td> <td>============        </td> <td>============         </td> <td>============       </td>
     * </tr>
     * <tr>
     * <td>         AUTO_PREP_CONE:                           </td> <td>51.0                </td> <td>4.0                  </td> <td>203.0              </td>
     * </tr>
     * <tr>
     * <td>         =========================                 </td> <td>============        </td> <td>============         </td> <td>============       </td>
     * </tr>
     * <tr>
     * <td>         ACQUIRE_SINGLE_SUBSTATION:                </td> <td>3.6                 </td> <td>2.0                  </td> <td>320.0              </td>
     * </tr>
     * <tr>
     * <td>         ACQUIRE_DOUBLE_SUBSTATION_CONE:           </td> <td>51.0                </td> <td>2.0                  </td> <td>193.5              </td>
     * </tr>
     * <tr>
     * <td>         ACQUIRE_DOUBLE_SUBSTATION_CUBE:           </td> <td>47.1                </td> <td>2.0                  </td> <td>203.7              </td>
     * </tr>
     * </table>
     * </p>
     */
    public enum ScoringPositions {
        STOWED(                        3.0,        4.0,        305.0),
        INTERMEDIATE_LOW(              9.5,        21.5,       275.0),

        SCORE_LOW_CUBE(                15.0,       13.0,       200.0),
        SCORE_MID_CUBE(                39.0,       6.0,        215.0), 
        SCORE_HIGH_CUBE(               51.0,       34.0,       203.0),

        AUTON_PREP_CUBE(               51.0,       4.0,        203.0),

        ACQUIRE_FLOOR_CUBE(            9.0,        23.0,       245.0),
        ACQUIRE_FLOOR_CONE_TIPPED(     8.0,        26.5,       260.0),
        ACQUIRE_FLOOR_CONE_UPRIGHT(    8.0,        19.6,       262.5),
        
        AUTON_URPIGHT_CONE(            8.5,        19.6,       261.5),

        SCORE_LOW_CONE(                15.0,       13.0,       200.0),
        SCORE_MID_CONE(                39.0,       6.0,        215.0), 
        SCORE_HIGH_CONE(               51.0,       34.0,       203.0),

        AUTON_PREP_CONE(               51.0,       4.0,        203.0),

        ACQUIRE_SINGLE_SUBSTATION(     3.6,        2.0,        320.0),
        ACQUIRE_DOUBLE_SUBSTATION_CONE(51.0,       2.0,        193.5),
        ACQUIRE_DOUBLE_SUBSTATION_CUBE(47.1,       2.0,        203.7);

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
                FieldConstants.FIELD_WIDTH.getAsMeters() - pose.getY(),
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
    private static BeakSwerveDrivetrain m_drive;
    private static Vision m_vision;
    private static UpperArm m_upperArm;
    private static LowerArm m_lowerArm;
    private static Wrist m_wrist;

    private static boolean m_climbMode = false;
    private static boolean m_scoreMode = false;
    private static boolean m_snappedMode = false;
    // private static boolean m_areTheLightsOn = false;

    /**
     * Turns off the CANdle
     */
    public static void killTheLights() {
        // m_areTheLightsOn = false;
        if (!m_climbMode) {
            m_leds.setBlank();
        }
    }

    /**
     * sets the robot mode to Orange (cone) mode
     */
    public static void becomeOrangeMode() {
        // m_areTheLightsOn = true;
        m_currentMode = GamePieceMode.ORANGE_CONE;
        m_leds.blink(Color.ORANGE).schedule();
    }

    /**
     * sets the robot mode to Purple (cube) mode
     */
    public static void becomePurpleMode() {
        // m_areTheLightsOn = true;
        m_currentMode = GamePieceMode.PURPLE_CUBE;
        m_leds.blink(Color.PURPLE).schedule();
    }

    // TODO: Javadoc ALL these
    public static Color getCurrentColor() {
        return m_leds.getColor();
    }

    public static boolean getFade() {
        return m_leds.getFade();
    }

    public static void setActive() {
        m_climbMode = false;
        m_leds.setBeaconState(false);
        m_scoreMode = false;
        if (m_leds.getMode() != CANdleMode.VICTORY_SPIN) {
            m_leds.setActive();
        }
        m_leds.setFade(false);
    }

    // TODO: celanup this stuff
    public static void toggleSnappedMode() {
        setSnappedMode(!m_snappedMode);
    }

    public static void setSnappedMode(boolean snapped) {
        m_leds.setBeaconState(snapped);
        m_snappedMode = snapped;

        checkAuxiliaryModes();
    }

    public static boolean getBeaconState() {
        return m_leds.getBeaconState();
    }

    public static boolean getSnapped() {
        return m_snappedMode;
    }

    public static void setIdle() {
        m_leds.setIdle().until(() -> m_leds.getMode() != CANdleMode.IDLE).ignoringDisable(true).schedule();
    }

    public static void toggleSlide() {
        if(m_leds.getMode() != CANdleMode.SLIDE) {
            m_leds.setSlide();
        } else {
            m_leds.setActive();
        }
    }

    public static void setFireWorkPlz() {
        if(m_leds.getMode() != CANdleMode.FIRE) {
            m_leds.setFireWorkPlz();
        } else {
            m_leds.setActive();
        }
    }

    public static void toggleVictorySpin() {
        switch (m_leds.getMode()) {
            case VICTORY_SPIN:
                m_leds.setActive();
                break;
            default:
                m_leds.setVictorySpin();
                break;
        }
    }

    public static CANdleMode getCANdleMode() {
        return m_leds.getMode();
    }

    /**
     * Set the robot to orange (cone) mode
     * 
     * @return A {@link Command} that changes the robot to orange mode.
     */
    public static Command orangeModeCommand() {
        return new InstantCommand(() -> {
            becomeOrangeMode();
            checkAuxiliaryModes();
        }, m_leds);
    }

    /**
     * Set the robot to purple (cube) mode
     * 
     * @return A {@link Command} that changes the robot to purple mode.
     */
    public static Command purpleModeCommand() {
        return new InstantCommand(() -> {
            becomePurpleMode();
            checkAuxiliaryModes();
        }, m_leds);
    }

    public static Command getModeCommand(GamePieceMode mode) {
        return mode == GamePieceMode.ORANGE_CONE ? orangeModeCommand() : purpleModeCommand();
    }

    // TODO: javadoc these
    public static void toggleScoreMode() {
        m_scoreMode = !m_scoreMode;
        SmartDashboard.putBoolean("Score Mode", m_scoreMode);

        m_leds.setFade(m_scoreMode);
    }

    public static void setScoreMode(boolean scoreMode) {
        m_scoreMode = scoreMode;
        m_leds.setFade(scoreMode);
    }

    public static void toggleClimbMode() {
        m_climbMode = !m_climbMode;
        m_leds.setBeaconState(m_climbMode);

        checkAuxiliaryModes();
    }

    public static void setClimbMode(boolean climb) {
        m_climbMode = climb;

        checkAuxiliaryModes();
    }

    /**
     * Check the status of the climb and auto align modes and blinks the necessary
     * color.
     */
    public static void checkAuxiliaryModes() {
        if (m_climbMode) {
            m_leds.setBeaconState(true);
            m_leds.setBeaconColor(Color.GREEN);
        } else if (m_snappedMode) {
            m_leds.setBeaconState(true);
            m_leds.setBeaconColor(Color.RED);
        } else {
            m_leds.setBeaconState(false);
            blinkCurrentColor();
        }
    }

    public static void checkAuxiliaryModesPeriodic() {
        if (m_climbMode) {
            m_leds.setBeaconState(true);
            m_leds.setBeaconColor(Color.GREEN);
        } else if (m_snappedMode) {
            m_leds.setBeaconState(true);
            m_leds.setBeaconColor(Color.RED);
        } else {
            m_leds.setBeaconState(false);
        }
    }

    /**
     * Blink the LEDs using the current mode.
     */
    public static void blinkCurrentColor() {
        switch (m_currentMode) {
            case ORANGE_CONE:
                m_leds.blink(LEDs.Color.ORANGE).schedule();
                break;
            case PURPLE_CUBE:
                m_leds.blink(LEDs.Color.PURPLE).schedule();
                break;
            default:
                m_leds.blink(LEDs.Color.ORANGE).schedule();
                break;
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

    public static void setNode(Node node) {
        m_currentNode = node;
    }

    /**
     * Run to the position of the currently set scoring node.
     * 
     * @param interruptCondition
     *            A condition that will stop the automatic alignment.
     * @return A {@link Command} to run to the set node position.
     */
    public static Command runToNodePosition(BooleanSupplier interruptCondition) {
        // Add measurements to the pose estimator before and after to ensure relative
        // accuracy
        return new ConditionalCommand(
            new GeneratePathWithArc(
                () -> DriverStation.getAlliance() == Alliance.Red ? m_currentNode.RedPose : m_currentNode.BluePose,
                interruptCondition,
                m_drive)
                    .withInterruptBehavior(InterruptionBehavior.kCancelSelf),
            Commands.none(),
            () -> m_scoreMode);
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

                setNode(nodeToSet.get(0));
            });
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

                setNode(nodeToSet.get(0));
            });
    }

    public static void addSubsystems(LEDs leds, BeakSwerveDrivetrain drivetrain, Vision vision, LowerArm lowerArm,
        UpperArm upperArm, Wrist wrist) {
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

    public static boolean getScoreMode() {
        return m_scoreMode;
    }

    // public static boolean getLightMode() {
    // return m_areTheLightsOn;
    // }

    public static Command runArms(ScoringPositions targetPos) {
        // return new RunArmsSafely(targetPos, m_lowerArm, m_upperArm, m_wrist);
        return new InstantCommand(() -> {
            if (targetPos.equals(ScoringPositions.ACQUIRE_SINGLE_SUBSTATION)) {
                toggleSlide();
            } else {
                if (m_leds.getMode() == CANdleMode.SLIDE) {
                    setActive();
                }
            }
        }).alongWith(new RunArmsWithPID(targetPos, m_lowerArm, m_upperArm, m_wrist));
    }
 //TODO
    public static void signalAcquisition() {
        if(getCANdleMode() == CANdleMode.ACTIVE) {
            m_leds.blinkWhite().schedule();
        } else {
            m_leds.blinkBeaconWhiteAndRed().schedule();
        }
    }

    /**
     * Get the forward-backward jerk of the robot.
     * @return The Y direction jerk.
     */
    public static double getJerk() {
        return m_drive.getJerk();
    }
}