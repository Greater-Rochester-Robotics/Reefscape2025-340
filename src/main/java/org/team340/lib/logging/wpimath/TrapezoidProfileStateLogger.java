package org.team340.lib.logging.wpimath;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

@CustomLoggerFor(TrapezoidProfile.State.class)
public class TrapezoidProfileStateLogger extends ClassSpecificLogger<TrapezoidProfile.State> {

    public TrapezoidProfileStateLogger() {
        super(TrapezoidProfile.State.class);
    }

    @Override
    public void update(EpilogueBackend backend, TrapezoidProfile.State state) {
        backend.log("position", state.position);
        backend.log("velocity", state.velocity);
    }
}
