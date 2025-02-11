package org.team340.lib.logging.phoenix.controls;

import com.ctre.phoenix6.controls.Follower;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(Follower.class)
public class FollowerLogger extends ClassSpecificLogger<Follower> {

    public FollowerLogger() {
        super(Follower.class);
    }

    @Override
    public void update(EpilogueBackend backend, Follower control) {
        // No-op
    }
}
