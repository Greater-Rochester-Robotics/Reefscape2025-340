package org.team340.lib.logging.phoenix.controls;

import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(DutyCycleOut.class)
public class DutyCycleOutLogger extends ClassSpecificLogger<DutyCycleOut> {

    public DutyCycleOutLogger() {
        super(DutyCycleOut.class);
    }

    @Override
    public void update(EpilogueBackend backend, DutyCycleOut control) {
        // No-op
    }
}
