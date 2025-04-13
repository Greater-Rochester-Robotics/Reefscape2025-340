package org.team340.robot.util;

import static org.photonvision.PhotonPoseEstimator.PoseStrategy.CONSTRAINED_SOLVEPNP;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.PNP_DISTANCE_TRIG_SOLVE;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.team340.lib.swerve.SwerveAPI.TimestampedPose;
import org.team340.lib.swerve.SwerveAPI.VisionMeasurement;
import org.team340.lib.util.Alliance;
import org.team340.robot.Constants.Cameras;
import org.team340.robot.util.PhotonPoseEstimator.ConstrainedSolvepnpParams;

/**
 * Manages all of the robot's cameras.
 */
@Logged
public final class VisionManager {

    private static VisionManager instance = null;

    public static VisionManager getInstance() {
        if (instance == null) instance = new VisionManager();
        return instance;
    }

    @NotLogged
    private final Camera[] cameras;

    private final AprilTagFieldLayout aprilTags;

    private final List<Pose2d> estimates = new ArrayList<>();
    private final List<Pose3d> targets = new ArrayList<>();

    private VisionManager() {
        aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        cameras = new Camera[] {
            new Camera("middle", Cameras.kMiddle),
            new Camera("left", Cameras.kLeft),
            new Camera("right", Cameras.kRight)
        };

        // Hit the undocumented Photon Turbo Button™
        // https://github.com/PhotonVision/photonvision/pull/1662
        NetworkTableInstance.getDefault().getBooleanTopic("/photonvision/use_new_cscore_frametime").publish().set(true);
    }

    /**
     * Resets cached measurements utilized by the pose estimators for seeding. It is
     * recommended to call this method after resetting the robot's pose or rotation.
     */
    public void reset() {
        for (var camera : cameras) camera.clearHeadingData();
    }

    /**
     * Gets unread results from all cameras.
     * @param poseHistory Robot pose estimates from the last robot cycle.
     */
    public VisionMeasurement[] getUnreadResults(List<TimestampedPose> poseHistory) {
        List<VisionMeasurement> measurements = new ArrayList<>();

        estimates.clear();
        targets.clear();

        for (var camera : cameras) {
            camera.addReferencePoses(poseHistory);
            camera.refresh(measurements, targets);
        }

        estimates.addAll(measurements.stream().map(m -> m.pose()).toList());
        return measurements.stream().toArray(VisionMeasurement[]::new);
    }

    private class Camera {

        private final PhotonCamera camera;
        private final PhotonPoseEstimator estimator;
        private final Optional<ConstrainedSolvepnpParams> constrainedPnpParams;
        private final Debouncer disabledDebounce = new Debouncer(5.0, DebounceType.kFalling);

        /**
         * Create a camera.
         * @param cameraName The configured name of the camera.
         * @param robotToCamera The {@link Transform3d} from the robot's center to the camera.
         */
        private Camera(String cameraName, Transform3d robotToCamera) {
            camera = new PhotonCamera(cameraName);
            estimator = new PhotonPoseEstimator(aprilTags, PNP_DISTANCE_TRIG_SOLVE, robotToCamera);
            constrainedPnpParams = Optional.of(new ConstrainedSolvepnpParams(true, 0.0));
        }

        /**
         * Clears all heading data in the buffer.
         */
        private void clearHeadingData() {
            estimator.clearHeadingData();
        }

        /**
         * Adds reference poses to be utilized by the Photon pose estimator.
         * @param poseHistory Robot pose estimates from the last robot cycle.
         */
        private void addReferencePoses(List<TimestampedPose> poseHistory) {
            for (var pose : poseHistory) {
                estimator.addHeadingData(pose.timestamp(), pose.pose().getRotation());
            }
        }

        /**
         * Refreshes the provided lists with new unread results from the camera. Note
         * that this method does not remove any elements from the supplied lists.
         * @param measurements A list of vision measurements to add to.
         * @param targets A list of targets to add to.
         */
        private void refresh(List<VisionMeasurement> measurements, List<Pose3d> targets) {
            // If we are disabled, use Constrained SolvePNP to estimate the robot's heading.
            // See https://github.com/Greater-Rochester-Robotics/Reefscape2025-340/issues/19
            boolean usingTrig = disabledDebounce.calculate(DriverStation.isEnabled());
            estimator.setPrimaryStrategy(usingTrig ? PNP_DISTANCE_TRIG_SOLVE : CONSTRAINED_SOLVEPNP);

            for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
                // Get an estimate from the PhotonPoseEstimator.
                var estimate = estimator.update(result, Optional.empty(), Optional.empty(), constrainedPnpParams);
                if (estimate.isEmpty() || estimate.get().targetsUsed.isEmpty()) continue;

                // Get the target AprilTag, and reject the measurement if the
                // tag is not configured to be utilized by the pose estimator.
                var target = estimate.get().targetsUsed.get(0);
                int id = target.fiducialId;
                if (!useTag(id)) continue;

                // Get the location of the tag on the field.
                var tagLocation = aprilTags.getTagPose(id);
                if (tagLocation.isEmpty()) continue;

                // Determine the distance from the camera to the tag.
                double distance = target.bestCameraToTarget.getTranslation().getNorm();

                // Calculate the pose estimation weights for X/Y location. As
                // distance increases, the tag is trusted exponentially less.
                double xyStd = (usingTrig ? 0.1 : 0.4) * distance * distance;

                // Calculate the angular pose estimation weight. If we're solving via trig, reject
                // the heading estimate to ensure the pose estimator doesn't "poison" itself with
                // essentially duplicate data. Otherwise, weight the estimate similar to X/Y.
                double angStd = (usingTrig ? 1e5 : 0.14) * distance * distance;

                // Push the measurement to the supplied measurements list.
                measurements.add(
                    new VisionMeasurement(
                        estimate.get().estimatedPose.toPose2d(),
                        estimate.get().timestampSeconds,
                        VecBuilder.fill(xyStd, xyStd, angStd)
                    )
                );

                // Push the location of the tag to the targets list for telemetry.
                targets.add(tagLocation.get());
            }
        }

        /**
         * Returns {@code true} if an AprilTag should be utilized.
         * @param id The ID of the AprilTag.
         */
        private boolean useTag(int id) {
            return Alliance.isBlue() ? (id >= 17 && id <= 22) : (id >= 6 && id <= 11);
        }
    }
}
