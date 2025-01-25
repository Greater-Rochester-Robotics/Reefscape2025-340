package org.team340.lib.logging.wpimath;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.trajectory.ExponentialProfile;

@CustomLoggerFor(ExponentialProfile.Constraints.class)
public class ExponentialProfileConstraintsLogger extends ClassSpecificLogger<ExponentialProfile.Constraints> {

    public ExponentialProfileConstraintsLogger() {
        super(ExponentialProfile.Constraints.class);
    }

    @Override
    public void update(EpilogueBackend backend, ExponentialProfile.Constraints constraints) {
        // No-op
    }
}
