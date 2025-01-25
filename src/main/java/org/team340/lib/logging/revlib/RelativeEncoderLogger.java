package org.team340.lib.logging.revlib;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(RelativeEncoder.class)
public class RelativeEncoderLogger extends ClassSpecificLogger<RelativeEncoder> {

    public RelativeEncoderLogger() {
        super(RelativeEncoder.class);
    }

    @Override
    public void update(EpilogueBackend backend, RelativeEncoder relativeEncoder) {
        backend.log("position", relativeEncoder.getPosition());
        backend.log("velocity", relativeEncoder.getVelocity());
    }
}
