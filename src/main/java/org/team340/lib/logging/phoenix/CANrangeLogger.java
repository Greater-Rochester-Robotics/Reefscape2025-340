package org.team340.lib.logging.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

@CustomLoggerFor(CANrange.class)
public class CANrangeLogger extends ClassSpecificLogger<CANrange> {

    private static final Map<CANrange, Consumer<EpilogueBackend>> cache = new HashMap<>();

    public CANrangeLogger() {
        super(CANrange.class);
    }

    @Override
    public void update(EpilogueBackend backend, CANrange canRange) {
        cache
            .computeIfAbsent(canRange, key -> {
                var ambientSignal = canRange.getAmbientSignal(false);
                var distance = canRange.getDistance(false);
                var isDetected = canRange.getIsDetected(false);

                BaseStatusSignal[] signals = { ambientSignal, distance, isDetected };

                return b -> {
                    BaseStatusSignal.refreshAll(signals);
                    b.log("ambientSignal", ambientSignal.getValueAsDouble());
                    b.log("distance", distance.getValueAsDouble());
                    b.log("isDetected", isDetected.getValue());
                };
            })
            .accept(backend);
    }
}
