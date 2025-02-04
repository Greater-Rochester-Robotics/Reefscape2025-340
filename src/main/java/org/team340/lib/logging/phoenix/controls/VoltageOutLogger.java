package org.team340.lib.logging.phoenix.controls;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(VoltageOut.class)
public class VoltageOutLogger extends ClassSpecificLogger<VoltageOut> {

    public VoltageOutLogger() {
        super(VoltageOut.class);
    }

    @Override
    public void update(EpilogueBackend backend, VoltageOut control) {
        // No-op
    }
}
