package frc.robot;

import java.util.HashMap;
import java.util.Map;

public class UpdatedArmPositions {
    public static final Map<String, double[]> updatedArmPosMap = new HashMap<String, double[]>();

    public UpdatedArmPositions() {
        updatedArmPosMap.put("STOWED", new double[] { 3.0, 4.0, 305.0 });
        updatedArmPosMap.put("INTERMEDIATE_LOW", new double[] { 9.5, 21.5, 275.0 });
        updatedArmPosMap.put("SCORE_LOW_CUBE", new double[] { 15.0, 13.0, 200.0 });
        updatedArmPosMap.put("SCORE_MID_CUBE", new double[] { 39.0, 6.0, 215.0 });
        updatedArmPosMap.put("SCORE_HIGH_CUBE", new double[] { 51.0, 34.0, 203.0 });
        updatedArmPosMap.put("AUTON_PREP_CUBE", new double[] { 51.0, 4.0, 203.0 });
        updatedArmPosMap.put("ACQUIRE_FLOOR_CUBE", new double[] { 9.0, 23.0, 245.0 });
        updatedArmPosMap.put("ACQUIRE_FLOOR_CONE_TIPPED", new double[] { 8.0, 26.5, 260.0 });
        updatedArmPosMap.put("ACQUIRE_FLOOR_CONE_UPRIGHT", new double[] { 8.0, 19.6, 262.5 });
        updatedArmPosMap.put("AUTON_UPRIGHT_CONE", new double[] { 8.5, 19.6, 261.5 });
        updatedArmPosMap.put("SCORE_LOW_CONE", new double[] { 15.0, 13.0, 200.0 });
        updatedArmPosMap.put("SCORE_MID_CONE", new double[] { 39.0, 6.0, 215.0 });
        updatedArmPosMap.put("SCORE_HIGH_CONE", new double[] { 51.0, 34.0, 203.0 });
        updatedArmPosMap.put("AUTON_PREP_CONE", new double[] { 51.0, 4.0, 203.0 });
        updatedArmPosMap.put("ACQUIRE_SINGLE_SUBSTATION", new double[] { 3.6, 2.0, 320.0 });
        updatedArmPosMap.put("ACQUIRE_DOUBLE_SUBSTATION_CONE", new double[] { 51.0, 2.0, 193.5 });
        updatedArmPosMap.put("ACQUIRE_DOUBLE_SUBSTATION_CUBE", new double[] { 47.1, 2.0, 203.7 });
    }
}
