package org.team340.lib.logging.phoenix.controls;

import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(VelocityVoltage.class)
public class VelocityVoltageLogger extends ClassSpecificLogger<VelocityVoltage> {

    public VelocityVoltageLogger() {
        super(VelocityVoltage.class);
    }

    @Override
    public void update(EpilogueBackend backend, VelocityVoltage control) {
        // No-op
    }
}
