package org.team340.lib.logging.phoenix.controls;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(VelocityTorqueCurrentFOC.class)
public class VelocityTorqueCurrentFOCLogger extends ClassSpecificLogger<VelocityTorqueCurrentFOC> {

    public VelocityTorqueCurrentFOCLogger() {
        super(VelocityTorqueCurrentFOC.class);
    }

    @Override
    public void update(EpilogueBackend backend, VelocityTorqueCurrentFOC control) {
        // No-op
    }
}
