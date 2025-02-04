package org.team340.lib.logging.phoenix.controls;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(MotionMagicVoltage.class)
public class MotionMagicVoltageLogger extends ClassSpecificLogger<MotionMagicVoltage> {

    public MotionMagicVoltageLogger() {
        super(MotionMagicVoltage.class);
    }

    @Override
    public void update(EpilogueBackend backend, MotionMagicVoltage control) {
        // No-op
    }
}
