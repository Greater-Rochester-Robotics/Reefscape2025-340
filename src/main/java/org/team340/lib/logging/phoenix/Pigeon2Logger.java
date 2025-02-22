package org.team340.lib.logging.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

@CustomLoggerFor(Pigeon2.class)
public class Pigeon2Logger extends ClassSpecificLogger<Pigeon2> {

    private static final Map<Pigeon2, Consumer<EpilogueBackend>> cache = new HashMap<>();

    public Pigeon2Logger() {
        super(Pigeon2.class);
    }

    @Override
    public void update(EpilogueBackend backend, Pigeon2 pigeon2) {
        cache
            .computeIfAbsent(pigeon2, key -> {
                var temperature = pigeon2.getTemperature(false);
                var yaw = pigeon2.getYaw(false);
                var pitch = pigeon2.getPitch(false);
                var roll = pigeon2.getRoll(false);

                BaseStatusSignal[] signals = { temperature, yaw, pitch, roll };

                return b -> {
                    BaseStatusSignal.refreshAll(signals);
                    b.log("temperature", temperature.getValueAsDouble());
                    b.log("yaw", yaw.getValueAsDouble());
                    b.log("pitch", pitch.getValueAsDouble());
                    b.log("roll", roll.getValueAsDouble());
                };
            })
            .accept(backend);
    }
}
