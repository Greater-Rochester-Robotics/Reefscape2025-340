package org.team340.lib.logging.wpilibj;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj.Encoder;

@CustomLoggerFor(Encoder.class)
public class EncoderLogger extends ClassSpecificLogger<Encoder> {

    public EncoderLogger() {
        super(Encoder.class);
    }

    @Override
    public void update(EpilogueBackend backend, Encoder encoder) {
        backend.log("distance", encoder.getDistance());
        backend.log("rate", encoder.getRate());
    }
}
