package org.team340.lib.logging.wpilibj;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj.AddressableLED;

@CustomLoggerFor(AddressableLED.class)
public class AddressableLEDLogger extends ClassSpecificLogger<AddressableLED> {

    public AddressableLEDLogger() {
        super(AddressableLED.class);
    }

    @Override
    public void update(EpilogueBackend backend, AddressableLED addressableLED) {
        // No-op
    }
}
