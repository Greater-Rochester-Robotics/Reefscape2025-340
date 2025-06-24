package org.team340.lib.logging.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Function;

@CustomLoggerFor(CANcoder.class)
public class CANcoderLogger extends ClassSpecificLogger<CANcoder> {

    private static final Map<CANcoder, Consumer<EpilogueBackend>> registry = new HashMap<>();
    private static final Function<CANcoder, Consumer<EpilogueBackend>> mappingFunction = cancoder -> {
        var position = cancoder.getPosition(false);
        var velocity = cancoder.getVelocity(false);

        BaseStatusSignal[] signals = { position, velocity };

        return backend -> {
            BaseStatusSignal.refreshAll(signals);
            backend.log("position", position.getValueAsDouble());
            backend.log("velocity", velocity.getValueAsDouble());
        };
    };

    public CANcoderLogger() {
        super(CANcoder.class);
    }

    @Override
    public void update(EpilogueBackend backend, CANcoder cancoder) {
        registry.computeIfAbsent(cancoder, mappingFunction).accept(backend);
    }
}
