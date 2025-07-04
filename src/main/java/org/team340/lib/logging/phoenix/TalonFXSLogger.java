package org.team340.lib.logging.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Function;

@CustomLoggerFor(TalonFXS.class)
public class TalonFXSLogger extends ClassSpecificLogger<TalonFXS> {

    private static final Map<TalonFXS, Consumer<EpilogueBackend>> registry = new HashMap<>();
    private static final Function<TalonFXS, Consumer<EpilogueBackend>> mappingFunction = talonFXS -> {
        var closedLoopReference = talonFXS.getClosedLoopReference(false);
        var deviceTemp = talonFXS.getDeviceTemp(false);
        var motorVoltage = talonFXS.getMotorVoltage(false);
        var position = talonFXS.getPosition(false);
        var statorCurrent = talonFXS.getStatorCurrent(false);
        var supplyCurrent = talonFXS.getSupplyCurrent(false);
        var velocity = talonFXS.getVelocity(false);

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

    public TalonFXSLogger() {
        super(TalonFXS.class);
    }

    @Override
    public void update(EpilogueBackend backend, TalonFXS talonFXS) {
        registry.computeIfAbsent(talonFXS, mappingFunction).accept(backend);
    }
}
