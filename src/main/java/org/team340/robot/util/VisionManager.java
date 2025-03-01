package org.team340.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.team340.lib.swerve.SwerveAPI.TimestampedYaw;
import org.team340.lib.swerve.SwerveAPI.VisionMeasurement;
import org.team340.lib.util.Alliance;
import org.team340.robot.Constants.Cameras;

/**
 * Manages all of the robot's cameras.
 */
@Logged(strategy = Strategy.OPT_IN)
public final class VisionManager {

    private static final AprilTagFields kField = AprilTagFields.k2025ReefscapeWelded;
    private static final PoseStrategy kStrategy = PoseStrategy.PNP_DISTANCE_TRIG_SOLVE;

    private static VisionManager instance = null;

    public static VisionManager getInstance() {
        if (instance == null) {
            instance = new VisionManager();
        }

        return instance;
    }

    private final AprilTagFieldLayout aprilTags;
    private final Camera[] cameras;

    private VisionManager() {
        aprilTags = AprilTagFieldLayout.loadField(kField);
        cameras = new Camera[] {
            new Camera("middle", Cameras.kMiddle),
            new Camera("left", Cameras.kLeft),
            new Camera("right", Cameras.kRight)
        };

        NetworkTableInstance.getDefault().getBooleanTopic("/photonvision/use_new_cscore_frametime").publish().set(true);
    }

    /**
     * Adds yaw measurements to be used for pose estimation.
     * @param yawMeasurements Robot yaw measurements since the last robot cycle.
     */
    public void addYawMeasurements(List<TimestampedYaw> yawMeasurements) {
        for (var camera : cameras) {
            camera.addYawMeasurements(yawMeasurements);
        }
    }

    /**
     * Gets unread results from all cameras.
     */
    public VisionEstimates getUnreadResults() {
        List<VisionMeasurement> measurements = new ArrayList<>();
        List<Pose3d> targets = new ArrayList<>();

        for (var camera : cameras) {
            var result = camera.getUnreadResults();
            measurements.addAll(result.measurements());
            targets.addAll(result.targets());
        }

        return new VisionEstimates(measurements, targets);
    }

    private class Camera {

        private final PhotonCamera camera;
        private final PhotonPoseEstimator estimator;

        /**
         * Create a camera.
         * @param cameraName The configured name of the camera.
         * @param robotToCamera The {@link Transform3d} from the robot's center to the camera.
         */
        private Camera(String cameraName, Transform3d robotToCamera) {
            camera = new PhotonCamera(cameraName);
            estimator = new PhotonPoseEstimator(aprilTags, kStrategy, robotToCamera);
        }

        /**
         * Adds yaw measurements to be used for pose estimation.
         * @param yawMeasurements Robot yaw measurements since the last robot cycle.
         */
        private void addYawMeasurements(List<TimestampedYaw> yawMeasurements) {
            for (var measurement : yawMeasurements) {
                estimator.addHeadingData(measurement.timestamp(), measurement.yaw());
            }
        }

        /**
         * Gets unread results from the camera.
         */
        private VisionEstimates getUnreadResults() {
            List<VisionMeasurement> measurements = new ArrayList<>();
            List<Pose3d> targets = new ArrayList<>();

            for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
                var estimate = estimator.update(result);
                if (estimate.isEmpty() || estimate.get().targetsUsed.isEmpty()) continue;

                var target = estimate.get().targetsUsed.get(0);
                int id = target.fiducialId;
                if (!useTag(id)) continue;

                var tagLocation = aprilTags.getTagPose(id);
                if (tagLocation.isEmpty()) continue;

                double distance = target.bestCameraToTarget.getTranslation().getNorm();
                double std = 0.1 * Math.pow(distance, 2.0);

                measurements.add(
                    new VisionMeasurement(
                        estimate.get().estimatedPose.toPose2d(),
                        estimate.get().timestampSeconds,
                        VecBuilder.fill(std, std, 1000.0)
                    )
                );

                targets.add(tagLocation.get());
            }

            return new VisionEstimates(measurements, targets);
        }

        /**
         * Returns {@code true} if an AprilTag should be utilized.
         * @param id The ID of the AprilTag.
         */
        private boolean useTag(int id) {
            return Alliance.isBlue() ? (id >= 17 && id <= 22) : (id >= 6 && id <= 11);
        }
    }

    public final record VisionEstimates(List<VisionMeasurement> measurements, List<Pose3d> targets) {
        /**
         * Returns all robot pose estimates in the calculated measurements.
         */
        public List<Pose2d> getPoses() {
            return measurements.stream().map(m -> m.visionPose()).toList();
        }
    }
}
