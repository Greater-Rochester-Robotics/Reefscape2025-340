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
                var accelerationX = pigeon2.getAccelerationX(false);
                var accelerationY = pigeon2.getAccelerationY(false);
                var accelerationZ = pigeon2.getAccelerationZ(false);
                var velocityX = pigeon2.getAngularVelocityXWorld(false);
                var velocityY = pigeon2.getAngularVelocityYWorld(false);
                var velocityZ = pigeon2.getAngularVelocityZWorld(false);
                var yaw = pigeon2.getYaw(false);
                var pitch = pigeon2.getPitch(false);
                var roll = pigeon2.getRoll(false);

                BaseStatusSignal[] signals = {
                    temperature,
                    accelerationX,
                    accelerationY,
                    accelerationZ,
                    velocityX,
                    velocityY,
                    velocityZ,
                    yaw,
                    pitch,
                    roll
                };

                return b -> {
                    BaseStatusSignal.refreshAll(signals);
                    b.log("temperature", temperature.getValueAsDouble());
                    b.log("accelerationX", accelerationX.getValueAsDouble());
                    b.log("accelerationY", accelerationY.getValueAsDouble());
                    b.log("accelerationZ", accelerationZ.getValueAsDouble());
                    b.log("velocityX", velocityX.getValueAsDouble());
                    b.log("velocityY", velocityY.getValueAsDouble());
                    b.log("velocityZ", velocityZ.getValueAsDouble());
                    b.log("yaw", yaw.getValueAsDouble());
                    b.log("pitch", pitch.getValueAsDouble());
                    b.log("roll", roll.getValueAsDouble());
                };
            })
            .accept(backend);
    }
}
