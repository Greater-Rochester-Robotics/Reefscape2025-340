package org.team340.lib.logging.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

@CustomLoggerFor(CANcoder.class)
public class CANcoderLogger extends ClassSpecificLogger<CANcoder> {

    private static final Map<CANcoder, Consumer<EpilogueBackend>> cache = new HashMap<>();

    public CANcoderLogger() {
        super(CANcoder.class);
    }

    @Override
    public void update(EpilogueBackend backend, CANcoder cancoder) {
        cache
            .computeIfAbsent(cancoder, key -> {
                var position = cancoder.getPosition(false);
                var velocity = cancoder.getVelocity(false);

                BaseStatusSignal[] signals = { position, velocity };

                return b -> {
                    BaseStatusSignal.refreshAll(signals);
                    b.log("position", position.getValueAsDouble());
                    b.log("velocity", velocity.getValueAsDouble());
                };
            })
            .accept(backend);
    }
}
