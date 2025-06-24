package org.team340.lib.logging.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Function;

@CustomLoggerFor(TalonFX.class)
public class TalonFXLogger extends ClassSpecificLogger<TalonFX> {

    private static final Map<TalonFX, Consumer<EpilogueBackend>> registry = new HashMap<>();
    private static final Function<TalonFX, Consumer<EpilogueBackend>> mappingFunction = talonFX -> {
        var closedLoopReference = talonFX.getClosedLoopReference(false);
        var deviceTemp = talonFX.getDeviceTemp(false);
        var motorVoltage = talonFX.getMotorVoltage(false);
        var position = talonFX.getPosition(false);
        var statorCurrent = talonFX.getStatorCurrent(false);
        var supplyCurrent = talonFX.getSupplyCurrent(false);
        var velocity = talonFX.getVelocity(false);

        BaseStatusSignal[] signals = {
            closedLoopReference,
            deviceTemp,
            motorVoltage,
            position,
            statorCurrent,
            supplyCurrent,
            velocity
        };

        return backend -> {
            BaseStatusSignal.refreshAll(signals);
            backend.log("closedLoopReference", closedLoopReference.getValueAsDouble());
            backend.log("deviceTemp", deviceTemp.getValueAsDouble());
            backend.log("motorVoltage", motorVoltage.getValueAsDouble());
            backend.log("position", position.getValueAsDouble());
            backend.log("statorCurrent", statorCurrent.getValueAsDouble());
            backend.log("supplyCurrent", supplyCurrent.getValueAsDouble());
            backend.log("velocity", velocity.getValueAsDouble());
        };
    };

    public TalonFXLogger() {
        super(TalonFX.class);
    }

    @Override
    public void update(EpilogueBackend backend, TalonFX talonFX) {
        registry.computeIfAbsent(talonFX, mappingFunction).accept(backend);
    }
}
