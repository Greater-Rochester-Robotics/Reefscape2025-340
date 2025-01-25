package org.team340.lib.logging.wpimath;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

@CustomLoggerFor(TrapezoidProfile.Constraints.class)
public class TrapezoidProfileConstraintsLogger extends ClassSpecificLogger<TrapezoidProfile.Constraints> {

    public TrapezoidProfileConstraintsLogger() {
        super(TrapezoidProfile.Constraints.class);
    }

    @Override
    public void update(EpilogueBackend backend, TrapezoidProfile.Constraints constraints) {
        // No-op
    }
}
