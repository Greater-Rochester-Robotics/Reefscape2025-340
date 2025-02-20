package org.team340.robot.util;

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
import org.team340.lib.swerve.SwerveAPI.VisionMeasurement;
import org.team340.robot.Constants.FieldConstants;

/**
 * Retrieves vision estimates from Photon cameras.
 */
@Logged(strategy = Strategy.OPT_IN)
public class VisionEstimator {

    public final record VisionEstimates(List<VisionMeasurement> measurements, List<Pose3d> targets) {
        /**
         * Returns all robot pose estimates in the calculated measurements.
         */
        public List<Pose2d> getPoses() {
            return measurements.stream().map(m -> m.visionPose()).toList();
        }
    }

    private static final PoseStrategy kStrategy = PoseStrategy.PNP_DISTANCE_TRIG_SOLVE;

    static {
        var kTurboButtonPub = NetworkTableInstance.getDefault()
            .getBooleanTopic("photonvision/use_new_cscore_frametime")
            .publish();
        kTurboButtonPub.set(true);
    }

    private final PhotonCamera camera;
    private final PhotonPoseEstimator estimator;

    /**
     * Create a vision estimator.
     * @param cameraName The configured name of the camera.
     * @param robotToCamera The {@link Transform3d} from the robot's center to the camera.
     */
    public VisionEstimator(String cameraName, Transform3d robotToCamera) {
        camera = new PhotonCamera(cameraName);
        estimator = new PhotonPoseEstimator(FieldConstants.kAprilTags, kStrategy, robotToCamera);
    }

    /**
     * Get all unread results from the camera.
     * @param robotPose The current estimated pose of the robot.
     * @param poseTimestamp The timestamp the pose was captured at.
     */
    public VisionEstimates getUnreadResults(Pose2d robotPose, double poseTimestamp) {
        List<VisionMeasurement> measurements = new ArrayList<>();
        List<Pose3d> targets = new ArrayList<>();

        estimator.addHeadingData(poseTimestamp, robotPose.getRotation());

        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            var estimate = estimator.update(result);
            if (estimate.isPresent()) {
                var target = estimate.get().targetsUsed.get(0);
                if (target == null) continue;

                int id = target.fiducialId;
                var tagLocation = FieldConstants.kAprilTags.getTagPose(id);
                if (tagLocation.isEmpty()) continue;

                // TODO no magic numbers
                double distance = target.bestCameraToTarget.getTranslation().getNorm();
                double std = 0.2 * Math.pow(distance * (isImportant(id) ? 0.5 : 1.0), 2.0);

                measurements.add(
                    new VisionMeasurement(
                        estimate.get().estimatedPose.toPose2d(),
                        estimate.get().timestampSeconds,
                        VecBuilder.fill(std, std, 100.0)
                    )
                );
                targets.add(tagLocation.get());
            }
        }

        return new VisionEstimates(measurements, targets);
    }

    /**
     * Returns whether april tags should be weighted higher
     * @param id april tag id
     * @return true if tag should be weighted higher
     */
    private boolean isImportant(int id) {
        // TODO replace with configurable id numbers
        return id == 3 || id == 4 || id == 7 || id == 8;
    }
}
