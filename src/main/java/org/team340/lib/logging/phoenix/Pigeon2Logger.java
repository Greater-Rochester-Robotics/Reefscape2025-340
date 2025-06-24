package org.team340.lib.logging.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Function;

@CustomLoggerFor(Pigeon2.class)
public class Pigeon2Logger extends ClassSpecificLogger<Pigeon2> {

    private static final Map<Pigeon2, Consumer<EpilogueBackend>> registry = new HashMap<>();
    private static final Function<Pigeon2, Consumer<EpilogueBackend>> mappingFunction = pigeon2 -> {
        var temperature = pigeon2.getTemperature(false);
        var yaw = pigeon2.getYaw(false);
        var pitch = pigeon2.getPitch(false);
        var roll = pigeon2.getRoll(false);

        BaseStatusSignal[] signals = { temperature, yaw, pitch, roll };

        return backend -> {
            BaseStatusSignal.refreshAll(signals);
            backend.log("temperature", temperature.getValueAsDouble());
            backend.log("yaw", yaw.getValueAsDouble());
            backend.log("pitch", pitch.getValueAsDouble());
            backend.log("roll", roll.getValueAsDouble());
        };
    };

    public Pigeon2Logger() {
        super(Pigeon2.class);
    }

    @Override
    public void update(EpilogueBackend backend, Pigeon2 pigeon2) {
        registry.computeIfAbsent(pigeon2, mappingFunction).accept(backend);
    }
}
