package org.team340.lib.logging.photonlib;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import org.photonvision.PhotonCamera;

@CustomLoggerFor(PhotonCamera.class)
public class PhotonCameraLogger extends ClassSpecificLogger<PhotonCamera> {

    public PhotonCameraLogger() {
        super(PhotonCamera.class);
    }

    @Override
    public void update(EpilogueBackend backend, PhotonCamera photonCamera) {
        // No-op
    }
}
