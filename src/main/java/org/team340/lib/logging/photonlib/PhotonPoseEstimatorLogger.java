package org.team340.lib.logging.photonlib;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import org.photonvision.PhotonPoseEstimator;

@CustomLoggerFor(PhotonPoseEstimator.class)
public class PhotonPoseEstimatorLogger extends ClassSpecificLogger<PhotonPoseEstimator> {

    public PhotonPoseEstimatorLogger() {
        super(PhotonPoseEstimator.class);
    }

    @Override
    public void update(EpilogueBackend backend, PhotonPoseEstimator photonPoseEstimator) {
        // No-op
    }
}
