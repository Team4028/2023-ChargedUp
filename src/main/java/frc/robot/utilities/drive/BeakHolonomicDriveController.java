package frc.robot.utilities.drive;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Custom version of a @HolonomicDriveController specifically for following
 * PathPlanner paths
 *
 * <p>
 * This controller adds the following functionality over the PathPlanner
 * version:
 * todo
 * </p>
 */
public class BeakHolonomicDriveController {
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;

    private final PIDController auxXController;
    private final PIDController auxYController;
    private final PIDController auxRotationController;

    private Translation2d translationError = new Translation2d();
    private Rotation2d rotationError = new Rotation2d();
    private Pose2d tolerance = new Pose2d();
    private boolean isEnabled = true;

    private Translation2d auxTranslationError = new Translation2d();
    private Rotation2d auxRotationError = new Rotation2d();

    private Translation2d largestAuxTranslationError = new Translation2d();
    private Rotation2d largestAuxRotationError = new Rotation2d();

    /**
     * Constructs a BeakHolonomicDriveController
     *
     * @param xController        A PID controller to respond to error in the
     *                           field-relative X direction
     * @param yController        A PID controller to respond to error in the
     *                           field-relative Y direction
     * @param rotationController A PID controller to respond to error in rotation
     */
    public BeakHolonomicDriveController(
            PIDController xController, PIDController yController, PIDController rotationController,
            PIDController auxXController, PIDController auxYController, PIDController auxRotationController) {
        this.xController = xController;
        this.yController = yController;
        this.rotationController = rotationController;

        this.auxXController = auxXController;
        this.auxYController = auxYController;
        this.auxRotationController = auxRotationController;

        // Auto-configure continuous input for rotation controller
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
        this.auxRotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Returns true if the pose error is within tolerance of the reference.
     *
     * @return True if the pose error is within tolerance of the reference.
     */
    public boolean atReference() {
        Translation2d translationTolerance = this.tolerance.getTranslation();
        Rotation2d rotationTolerance = this.tolerance.getRotation();

        return Math.abs(this.auxTranslationError.getX()) < translationTolerance.getX()
                && Math.abs(this.auxTranslationError.getY()) < translationTolerance.getY()
                && Math.abs(this.auxRotationError.getRadians()) < rotationTolerance.getRadians();
    }

    /**
     * Sets the pose error whic is considered tolerance for use with atReference()
     *
     * @param tolerance The pose error which is tolerable
     */
    public void setTolerance(Pose2d tolerance) {
        this.tolerance = tolerance;
    }

    /**
     * Enables and disables the controller for troubleshooting. When calculate() is
     * called on a
     * disabled controller, only feedforward values are returned.
     *
     * @param enabled If the controller is enabled or not
     */
    public void setEnabled(boolean enabled) {
        this.isEnabled = enabled;
    }

    /**
     * Return the weighting coefficient for the controller and feed forward outputs.
     * 
     * @return The coefficient. Multiply the primary output by this weight, and the
     *         aux output by (1 - weight).
     */
    public double getWeight() {
        if (auxRotationError.equals(rotationError) && auxTranslationError.equals(translationError)) {
            return 1.;
        }
        SmartDashboard.putNumber("rot error", (auxRotationError.getDegrees()));
        SmartDashboard.putNumber("tran error", (auxTranslationError.getNorm()));
        SmartDashboard.putNumber("rot error initial", (largestAuxRotationError.getDegrees()));
        SmartDashboard.putNumber("tran error initial", (largestAuxTranslationError.getNorm()));
        SmartDashboard.putNumber("rot error percent",
                (auxRotationError.getDegrees() / largestAuxRotationError.getDegrees()));
        SmartDashboard.putNumber("tran error percent",
                (auxTranslationError.getNorm() / largestAuxTranslationError.getNorm()));
        return MathUtil.clamp(((auxRotationError.getDegrees() / largestAuxRotationError.getDegrees())
                + (auxTranslationError.getNorm() / largestAuxTranslationError.getNorm())) / 2, 0., 1.);
    }

    /**
     * Calculates the next output of the holonomic drive controller
     *
     * @param currentPose    The current pose
     * @param referenceState The desired trajectory state
     * @return The next output of the holonomic drive controller
     */
    public ChassisSpeeds calculate(Pose2d currentPose, PathPlannerState referenceState) {
        double xFF = referenceState.velocityMetersPerSecond * referenceState.poseMeters.getRotation().getCos();
        double yFF = referenceState.velocityMetersPerSecond * referenceState.poseMeters.getRotation().getSin();
        double rotationFF = referenceState.holonomicAngularVelocityRadPerSec;

        this.translationError = referenceState.poseMeters.relativeTo(currentPose).getTranslation();
        this.rotationError = referenceState.holonomicRotation.minus(currentPose.getRotation());

        if (!this.isEnabled) {
            return ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, rotationFF, currentPose.getRotation());
        }

        double xFeedback = this.xController.calculate(currentPose.getX(), referenceState.poseMeters.getX());
        double yFeedback = this.yController.calculate(currentPose.getY(), referenceState.poseMeters.getY());
        double rotationFeedback = this.rotationController.calculate(
                currentPose.getRotation().getRadians(), referenceState.holonomicRotation.getRadians());

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback, currentPose.getRotation());
    }

    /**
     * Calculates the next output of the holonomic drive controller
     *
     * @param currentPose       The current pose
     * @param referenceState    The desired trajectory state
     * @param auxReferenceState The desired trajectory state of the auxiliary
     *                          trajectory
     * @return The next output of the holonomic drive controller
     */
    public ChassisSpeeds calculate(Pose2d currentPose, PathPlannerState referenceState,
            PathPlannerState auxReferenceState) {
        double primaryWeight = getWeight();
        double auxWeight = 1 - primaryWeight;

        this.translationError = referenceState.poseMeters.relativeTo(currentPose).getTranslation();
        this.rotationError = referenceState.holonomicRotation.minus(currentPose.getRotation());

        double xFF, yFF, rotationFF;

        double primaryXFF = referenceState.velocityMetersPerSecond * referenceState.poseMeters.getRotation().getCos();
        double primaryYFF = referenceState.velocityMetersPerSecond * referenceState.poseMeters.getRotation().getSin();
        double primaryRotationFF = referenceState.holonomicAngularVelocityRadPerSec;

        double xFeedback, yFeedback, rotationFeedback;

        double primaryXFeedback = this.xController.calculate(currentPose.getX(), referenceState.poseMeters.getX());
        double primaryYFeedback = this.yController.calculate(currentPose.getY(), referenceState.poseMeters.getY());
        double primaryRotationFeedback = this.rotationController.calculate(
                currentPose.getRotation().getRadians(), referenceState.holonomicRotation.getRadians());

        SmartDashboard.putBoolean("Bruh?",
                auxReferenceState == null || auxReferenceState.equals(new PathPlannerState()));

        if (auxReferenceState == null || auxReferenceState.equals(new PathPlannerState())) {
            // auxReferenceState = referenceState;
            this.auxTranslationError = this.translationError;
            this.auxRotationError = this.rotationError;

            return this.calculate(currentPose, referenceState);
        } else {
            this.auxTranslationError = auxReferenceState.poseMeters.relativeTo(currentPose).getTranslation();
            this.auxRotationError = auxReferenceState.holonomicRotation.minus(currentPose.getRotation());

            if (this.auxTranslationError.getNorm() > this.largestAuxTranslationError.getNorm()) {
                this.largestAuxTranslationError = this.auxTranslationError;
            }

            if (this.auxRotationError.getDegrees() > this.largestAuxRotationError.getDegrees()) {
                this.largestAuxRotationError = this.auxRotationError;
            }

            double auxXFF = auxReferenceState.velocityMetersPerSecond
                    * auxReferenceState.poseMeters.getRotation().getCos();
            double auxYFF = auxReferenceState.velocityMetersPerSecond
                    * auxReferenceState.poseMeters.getRotation().getSin();
            double auxRotationFF = auxReferenceState.holonomicAngularVelocityRadPerSec;

            xFF = (primaryXFF * primaryWeight) + (auxXFF * auxWeight);
            yFF = (primaryYFF * primaryWeight) + (auxYFF * auxWeight);
            rotationFF = (primaryRotationFF * primaryWeight) + (auxRotationFF * auxWeight);

            double auxXFeedback = this.auxXController.calculate(currentPose.getX(),
                    auxReferenceState.poseMeters.getX());
            double auxYFeedback = this.auxYController.calculate(currentPose.getY(),
                    auxReferenceState.poseMeters.getY());
            double auxRotationFeedback = this.auxRotationController.calculate(
                    currentPose.getRotation().getRadians(), auxReferenceState.holonomicRotation.getRadians());

            xFeedback = (primaryXFeedback * primaryWeight) + (auxXFeedback * auxWeight);
            yFeedback = (primaryYFeedback * primaryWeight) + (auxYFeedback * auxWeight);
            rotationFeedback = (primaryRotationFeedback * primaryWeight) + (auxRotationFeedback * auxWeight);
        }

        if (!this.isEnabled) {
            return ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, rotationFF, currentPose.getRotation());
        }

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback, currentPose.getRotation());
    }
}
