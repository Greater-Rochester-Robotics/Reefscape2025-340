package org.team340.lib.logging.wpimath;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.trajectory.ExponentialProfile;

@CustomLoggerFor(ExponentialProfile.class)
public class ExponentialProfileLogger extends ClassSpecificLogger<ExponentialProfile> {

    public ExponentialProfileLogger() {
        super(ExponentialProfile.class);
    }

    @Override
    public void update(EpilogueBackend backend, ExponentialProfile profile) {
        // No-op
    }
}
