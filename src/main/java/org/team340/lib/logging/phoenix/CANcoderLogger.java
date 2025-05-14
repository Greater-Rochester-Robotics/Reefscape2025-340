package org.team340.lib.logging.phoenix;

import static edu.wpi.first.util.struct.StructGenerator.genEnum;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.util.struct.Struct;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

@CustomLoggerFor(CANcoder.class)
public class CANcoderLogger extends ClassSpecificLogger<CANcoder> {

    private static final Map<CANcoder, Consumer<EpilogueBackend>> cache = new HashMap<>();
    private static final Struct<MagnetHealthValue> magnetHealthStruct = genEnum(MagnetHealthValue.class);

    public CANcoderLogger() {
        super(CANcoder.class);
    }

    @Override
    public void update(EpilogueBackend backend, CANcoder cancoder) {
        cache
            .computeIfAbsent(cancoder, key -> {
                var absolutePosition = cancoder.getAbsolutePosition(false);
                var magnetHealth = cancoder.getMagnetHealth(false);
                var position = cancoder.getPosition(false);
                var velocity = cancoder.getVelocity(false);

                BaseStatusSignal[] signals = { absolutePosition, magnetHealth, position, velocity };

                return b -> {
                    BaseStatusSignal.refreshAll(signals);
                    b.log("absolutePosition", absolutePosition.getValueAsDouble());
                    b.log("magnetHealth", magnetHealth.getValue(), magnetHealthStruct);
                    b.log("position", position.getValueAsDouble());
                    b.log("velocity", velocity.getValueAsDouble());
                };
            })
            .accept(backend);
    }
}
