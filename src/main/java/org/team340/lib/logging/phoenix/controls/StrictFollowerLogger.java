package org.team340.lib.logging.phoenix.controls;

import com.ctre.phoenix6.controls.StrictFollower;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(StrictFollower.class)
public class StrictFollowerLogger extends ClassSpecificLogger<StrictFollower> {

    public StrictFollowerLogger() {
        super(StrictFollower.class);
    }

    @Override
    public void update(EpilogueBackend backend, StrictFollower control) {
        // No-op
    }
}
