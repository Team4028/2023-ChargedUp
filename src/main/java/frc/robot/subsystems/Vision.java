// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Vision {
    private final PhotonCamera m_camera;
    private final AprilTagFieldLayout m_layout;
    private final PhotonPoseEstimator m_poseEstimator;

    private double m_latestLatency = 0.;
    private double m_latestTimestamp = 0.;

    private final Transform3d m_camToRobot;

    /**
     * Create a new Vision system.
     * 
     * @param cameraName
     *            The name of the camera in the PhotonVision UI.
     * @param robotToCam
     *            The transform from the robot to the camera.
     */
    public Vision(String cameraName, Transform3d robotToCam) {
        m_camera = new PhotonCamera(cameraName);

        m_camToRobot = robotToCam;

        try {
            m_layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            System.out.println(DriverStation.getAlliance());
        } catch (IOException err) {
            throw new RuntimeException();
        }

        m_poseEstimator = new PhotonPoseEstimator(m_layout, PoseStrategy.MULTI_TAG_PNP, m_camera, robotToCam);
        m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
    }

    /**
     * Create a new Vision subsystem.
     * 
     * @param cameraName
     *            The name of the camera in the PhotonVision UI.
     * @param robotToCam
     *            The transform from the robot to the camera.
     */
    public Vision(String cameraName, Pose3d robotToCam) {
        this(cameraName, robotToCam.minus(new Pose3d()));
    }

    public void togglePipeline() {
        m_camera.setPipelineIndex(m_camera.getPipelineIndex() == 0 ? 1 : 0);
    }

    /**
     * This method must be called every time a vision measurement is used.
     */
    public void checkAlliance() {
        m_layout.setOrigin(DriverStation.getAlliance() == Alliance.Red //
            ? OriginPosition.kRedAllianceWallRightSide
            : OriginPosition.kBlueAllianceWallRightSide);
    }

    /**
     * Get the best target the camera can see.
     * 
     * @return The best (lowest ambiguity) {@link PhotonTrackedTarget} in range.
     */
    public PhotonTrackedTarget getBestTarget() {
        List<PhotonTrackedTarget> targets = getTargets();

        targets.sort((targ1, targ2) -> {
            // descending sort
            return Double.compare(targ2.getPoseAmbiguity(), targ1.getPoseAmbiguity());
        });

        return targets.get(0);
    }

    /**
     * Get all targets from the camera.
     * 
     * @return The target data, or a blank list if none are found.
     */
    public List<PhotonTrackedTarget> getTargets() {
        PhotonPipelineResult result = m_camera.getLatestResult();

        m_latestLatency = result.getLatencyMillis() / 1000.;
        m_latestTimestamp = result.getTimestampSeconds();

        return result.getTargets();
    }

    /**
     * Get targets from the camera under the pose ambiguity threshold.
     * 
     * @return A list of {@link PhotonTrackedTarget}s filtered to remove
     *         high-ambiguity tags.
     */
    public List<PhotonTrackedTarget> getFilteredTargets() {
        List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();

        for (PhotonTrackedTarget target : getTargets())
            if (target.getPoseAmbiguity() < 0.45) {
                targets.add(target);
            }

        return targets;
    }

    /**
     * Get a pipeline result with really bad targets removed.
     * 
     * @return A {@link PhotonPipelineResult} with bad targets filtered out.
     */
    public PhotonPipelineResult getFilteredResult() {
        List<PhotonTrackedTarget> targets = getFilteredTargets();

        PhotonPipelineResult result = new PhotonPipelineResult(m_latestLatency * 1000., targets);
        result.setTimestampSeconds(m_latestTimestamp);

        return result;
    }

    /**
     * testing
     */
    public void logRawPoses() {
        List<PhotonTrackedTarget> targets = getTargets();
        PhotonTrackedTarget bestTarget = getBestTarget();
        
        Logger logger = Logger.getInstance();

        for (PhotonTrackedTarget target : targets) {
            int id = target.getFiducialId();
            Pose3d tagPose = m_layout.getTagPose(id).get();

            Pose3d estimatedPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), tagPose, m_camToRobot);

            logger.recordOutput("Target ID " + id, estimatedPose.toPose2d());
            if (target.equals(bestTarget)) {
                logger.recordOutput("Best Target", estimatedPose.toPose2d());
            }
        }
    }

    /**
     * Gets the best estimated pose of the robot in the current frame.
     * 
     * @param pose
     *            The current pose of the robot.
     * @return An {@link EstimatedRobotPose} of the robot.
     */
    public EstimatedRobotPose getLatestEstimatedRobotPose(Pose2d pose) {
        PhotonPipelineResult result = m_camera.getLatestResult();

        if (result.getTargets().size() == 1) {
            result = getFilteredResult();
        }

        m_poseEstimator.setReferencePose(pose);
        Optional<EstimatedRobotPose> estimatedRobotPose = m_poseEstimator.update();

        boolean poseExists = pose != null;
        boolean estimatedExists = estimatedRobotPose.isPresent();

        Pose3d estimatedPose = estimatedExists ? estimatedRobotPose.get().estimatedPose : new Pose3d();

        // Prefer estimated translation, but use passed-in translation if unavailable
        Translation2d finalTranslation = (estimatedExists
            ? estimatedPose.getTranslation().toTranslation2d()
            : (poseExists ? pose.getTranslation() : new Translation2d()));

        // Prefer passed-in rotation, but use estimated rotation if unavailable
        Rotation2d finalRotation = (poseExists ? pose.getRotation()
            : (estimatedExists ? estimatedPose.getRotation().toRotation2d() : new Rotation2d()));

        Pose2d finalPose = new Pose2d(finalTranslation, finalRotation);

        return new EstimatedRobotPose(new Pose3d(finalPose), result.getTimestampSeconds(), result.getTargets());
    }

    /**
     * Gets the pose of a target.
     * 
     * @param robotPose
     *            The current robot pose.
     * @param offset
     *            The offset of the desired pose from the target. Positive is
     *            backwards (X) and right (Y).
     * @return The pose of the specified offset from the target.
     */
    public Pose2d getTargetPose(Pose2d robotPose, Transform3d offset) {
        PhotonTrackedTarget target = getBestTarget();

        if (target != null) {
            Transform3d cameraToTarget = target.getBestCameraToTarget();

            Transform3d targetOffset = cameraToTarget.plus(offset);

            Pose3d pose = new Pose3d(robotPose);

            Pose3d scoringPose = pose.plus(targetOffset);

            // WARNING: The following code is scuffed. Please proceed with caution.
            Pose2d newPose = scoringPose.toPose2d();

            Rotation2d newRotation = Rotation2d
                .fromDegrees(newPose.getRotation().getDegrees());

            Pose2d finalPose = new Pose2d(newPose.getTranslation(), newRotation).plus(
                new Transform2d(
                    m_camToRobot.getTranslation().toTranslation2d(),
                    m_camToRobot.getRotation().toRotation2d()));

            return finalPose;
        }

        return robotPose;
    }
}
