package org.team340.lib.logging.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

@CustomLoggerFor(TalonFX.class)
public class TalonFXLogger extends ClassSpecificLogger<TalonFX> {

    private static final Map<TalonFX, Consumer<EpilogueBackend>> cache = new HashMap<>();

    public TalonFXLogger() {
        super(TalonFX.class);
    }

    @Override
    public void update(EpilogueBackend backend, TalonFX talonFX) {
        cache
            .computeIfAbsent(talonFX, key -> {
                var closedLoopReference = talonFX.getClosedLoopReference(false);
                var deviceTemp = talonFX.getDeviceTemp(false);
                var motorVoltage = talonFX.getMotorVoltage(false);
                var position = talonFX.getPosition(false);
                var statorCurrent = talonFX.getStatorCurrent(false);
                var velocity = talonFX.getVelocity(false);

                BaseStatusSignal[] signals = {
                    closedLoopReference,
                    deviceTemp,
                    motorVoltage,
                    position,
                    statorCurrent,
                    velocity
                };

                return b -> {
                    BaseStatusSignal.refreshAll(signals);
                    b.log("closedLoopReference", closedLoopReference.getValueAsDouble());
                    b.log("deviceTemp", deviceTemp.getValueAsDouble());
                    b.log("motorVoltage", motorVoltage.getValueAsDouble());
                    b.log("position", position.getValueAsDouble());
                    b.log("statorCurrent", statorCurrent.getValueAsDouble());
                    b.log("velocity", velocity.getValueAsDouble());
                };
            })
            .accept(backend);
    }
}
