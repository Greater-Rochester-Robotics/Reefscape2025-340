package org.team340.lib.logging.reduxlib;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandgyro.CanandgyroStatus;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(Canandgyro.class)
public class CanandgyroLogger extends ClassSpecificLogger<Canandgyro> {

    public CanandgyroLogger() {
        super(Canandgyro.class);
    }

    @Override
    public void update(EpilogueBackend backend, Canandgyro canandgyro) {
        backend.log("yaw", canandgyro.getYaw());
        backend.log("pitch", canandgyro.getPitch());
        backend.log("roll", canandgyro.getRoll());
        backend.log("connected", canandgyro.isConnected());
        backend.log("status", canandgyro.getStatus(), CanandgyroStatus.struct);
    }
}
