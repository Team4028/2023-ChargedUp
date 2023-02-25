// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final PhotonCamera m_camera;
    private final AprilTagFieldLayout m_layout;

    private double m_latestLatency;

    private final Pose3d m_camToRobot;
    private final boolean m_inverted;

    /**
     * Creates a new Vision.
     * 
     * @param inverted
     *            true for apriltags and false for others.
     */
    public Vision(String cameraName, Pose3d camToRobot, boolean inverted) {
        m_camera = new PhotonCamera(cameraName);

        m_camToRobot = camToRobot;
        m_inverted = inverted;

        try {
            m_layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException err) {
            throw new RuntimeException();
        }
    }

    public void togglePipeline() {
        m_camera.setPipelineIndex(m_camera.getPipelineIndex() == 0 ? 1 : 0);
    }

    /**
     * Get the best target from the camera.
     * 
     * @return The target data, or null if none are found.
     */
    public PhotonTrackedTarget getBestTarget() {
        PhotonPipelineResult result = m_camera.getLatestResult();

        m_latestLatency = result.getLatencyMillis() / 1000.;

        boolean hasTarget = result.hasTargets();

        PhotonTrackedTarget target = null;

        if (hasTarget) {
            target = result.getBestTarget();
        }

        return target;
    }

    public Pose2d getLatestEstimatedRobotPose() {
        PhotonTrackedTarget target = getBestTarget();

        if (target != null) {
            Transform3d cameraToTarget = target.getBestCameraToTarget();

            Optional<Pose3d> tagPose = m_layout.getTagPose(target.getFiducialId());

            Transform3d camToRobot = new Transform3d();

            if (tagPose.isPresent()) {
                Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, tagPose.get(), camToRobot);
                return robotPose.toPose2d();
            }
        }
        return new Pose2d();
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
                .fromDegrees(newPose.getRotation().getDegrees() - (m_inverted ? 180. : 0.));

            Pose2d finalPose = new Pose2d(newPose.getTranslation(), newRotation).plus(
                new Transform2d(
                    m_camToRobot.getTranslation().toTranslation2d(),
                    m_camToRobot.getRotation().toRotation2d()));

            Field2d field = new Field2d();
            field.setRobotPose(finalPose);
            SmartDashboard.putData("Vision Desired Pose", field);
            return finalPose;
        }

        return robotPose;
    }

    public double getLatestLatency() {
        return m_latestLatency;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
