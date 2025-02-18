package org.team340.lib.logging.wpimath;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.filter.Debouncer;

@CustomLoggerFor(Debouncer.class)
public class DebouncerLogger extends ClassSpecificLogger<Debouncer> {

    public DebouncerLogger() {
        super(Debouncer.class);
    }

    @Override
    public void update(EpilogueBackend backend, Debouncer debouncer) {
        // No-op
    }
}
