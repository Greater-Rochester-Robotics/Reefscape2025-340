package org.team340.lib.logging.wpilibj;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

@CustomLoggerFor(AddressableLEDBuffer.class)
public class AddressableLEDBufferLogger extends ClassSpecificLogger<AddressableLEDBuffer> {

    public AddressableLEDBufferLogger() {
        super(AddressableLEDBuffer.class);
    }

    @Override
    public void update(EpilogueBackend backend, AddressableLEDBuffer buffer) {
        // No-op
    }
}
