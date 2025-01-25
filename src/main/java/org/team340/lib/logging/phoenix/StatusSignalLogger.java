package org.team340.lib.logging.phoenix;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(StatusSignal.class)
public class StatusSignalLogger extends ClassSpecificLogger<StatusSignal<?>> {

    @SuppressWarnings({ "unchecked", "rawtypes" })
    public StatusSignalLogger() {
        super((Class) StatusSignal.class);
    }

    @Override
    public void update(EpilogueBackend backend, StatusSignal<?> signal) {}
}
