package org.team340.lib.logging.phoenix;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(CANBus.class)
public class CANBusLogger extends ClassSpecificLogger<CANBus> {

    public CANBusLogger() {
        super(CANBus.class);
    }

    @Override
    public void update(EpilogueBackend backend, CANBus canBus) {
        // No-op
    }
}
