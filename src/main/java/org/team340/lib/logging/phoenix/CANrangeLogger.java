package org.team340.lib.logging.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Function;

@CustomLoggerFor(CANrange.class)
public class CANrangeLogger extends ClassSpecificLogger<CANrange> {

    private static final Map<CANrange, Consumer<EpilogueBackend>> registry = new HashMap<>();
    private static final Function<CANrange, Consumer<EpilogueBackend>> mappingFunction = canrange -> {
        var ambientSignal = canrange.getAmbientSignal(false);
        var distance = canrange.getDistance(false);
        var isDetected = canrange.getIsDetected(false);

        BaseStatusSignal[] signals = { ambientSignal, distance, isDetected };

        return backend -> {
            BaseStatusSignal.refreshAll(signals);
            backend.log("ambientSignal", ambientSignal.getValueAsDouble());
            backend.log("distance", distance.getValueAsDouble());
            backend.log("isDetected", isDetected.getValue());
        };
    };

    public CANrangeLogger() {
        super(CANrange.class);
    }

    @Override
    public void update(EpilogueBackend backend, CANrange canrange) {
        registry.computeIfAbsent(canrange, mappingFunction).accept(backend);
    }
}
