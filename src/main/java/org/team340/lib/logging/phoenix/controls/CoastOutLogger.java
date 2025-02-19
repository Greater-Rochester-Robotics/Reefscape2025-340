package org.team340.lib.logging.phoenix.controls;

import com.ctre.phoenix6.controls.CoastOut;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(CoastOut.class)
public class CoastOutLogger extends ClassSpecificLogger<CoastOut> {

    public CoastOutLogger() {
        super(CoastOut.class);
    }

    @Override
    public void update(EpilogueBackend backend, CoastOut control) {
        // No-op
    }
}
