package org.team340.lib.logging.phoenix.controls;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(TorqueCurrentFOC.class)
public class TorqueCurrentFOCLogger extends ClassSpecificLogger<TorqueCurrentFOC> {

    public TorqueCurrentFOCLogger() {
        super(TorqueCurrentFOC.class);
    }

    @Override
    public void update(EpilogueBackend backend, TorqueCurrentFOC control) {
        // No-op
    }
}
