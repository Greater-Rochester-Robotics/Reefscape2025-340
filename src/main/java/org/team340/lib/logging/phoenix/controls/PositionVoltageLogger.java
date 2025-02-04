package org.team340.lib.logging.phoenix.controls;

import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(PositionVoltage.class)
public class PositionVoltageLogger extends ClassSpecificLogger<PositionVoltage> {

    public PositionVoltageLogger() {
        super(PositionVoltage.class);
    }

    @Override
    public void update(EpilogueBackend backend, PositionVoltage control) {
        // No-op
    }
}
