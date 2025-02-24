package org.team340.lib.logging.phoenix.controls;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(DynamicMotionMagicVoltage.class)
public class DynamicMotionMagicVoltageLogger extends ClassSpecificLogger<DynamicMotionMagicVoltage> {

    public DynamicMotionMagicVoltageLogger() {
        super(DynamicMotionMagicVoltage.class);
    }

    @Override
    public void update(EpilogueBackend backend, DynamicMotionMagicVoltage control) {
        // No-op
    }
}
