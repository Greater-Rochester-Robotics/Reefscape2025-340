package org.team340.lib.logging.wpimath;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.trajectory.ExponentialProfile;

@CustomLoggerFor(ExponentialProfile.State.class)
public class ExponentialProfileStateLogger extends ClassSpecificLogger<ExponentialProfile.State> {

    public ExponentialProfileStateLogger() {
        super(ExponentialProfile.State.class);
    }

    @Override
    public void update(EpilogueBackend backend, ExponentialProfile.State state) {
        backend.log("position", state.position);
        backend.log("velocity", state.velocity);
    }
}
