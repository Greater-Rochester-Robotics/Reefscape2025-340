package org.team340.lib.logging.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

@CustomLoggerFor(TalonFXS.class)
public class TalonFXSLogger extends ClassSpecificLogger<TalonFXS> {

    private static final Map<TalonFXS, Consumer<EpilogueBackend>> cache = new HashMap<>();

    public TalonFXSLogger() {
        super(TalonFXS.class);
    }

    @Override
    public void update(EpilogueBackend backend, TalonFXS talonFXS) {
        cache
            .computeIfAbsent(talonFXS, key -> {
                var acceleration = talonFXS.getAcceleration(false);
                var closedLoopError = talonFXS.getClosedLoopError(false);
                var closedLoopOutput = talonFXS.getClosedLoopOutput(false);
                var closedLoopReference = talonFXS.getClosedLoopReference(false);
                var deviceTemp = talonFXS.getDeviceTemp(false);
                var motorVoltage = talonFXS.getMotorVoltage(false);
                var position = talonFXS.getPosition(false);
                var statorCurrent = talonFXS.getStatorCurrent(false);
                var supplyCurrent = talonFXS.getSupplyCurrent(false);
                var supplyVoltage = talonFXS.getSupplyVoltage(false);
                var velocity = talonFXS.getVelocity(false);

                BaseStatusSignal[] signals = {
                    acceleration,
                    closedLoopError,
                    closedLoopOutput,
                    closedLoopReference,
                    deviceTemp,
                    motorVoltage,
                    position,
                    statorCurrent,
                    supplyCurrent,
                    supplyVoltage,
                    velocity
                };

                return b -> {
                    BaseStatusSignal.refreshAll(signals);
                    b.log("acceleration", acceleration.getValueAsDouble());
                    b.log("closedLoopError", closedLoopError.getValueAsDouble());
                    b.log("closedLoopOutput", closedLoopOutput.getValueAsDouble());
                    b.log("closedLoopReference", closedLoopReference.getValueAsDouble());
                    b.log("deviceTemp", deviceTemp.getValueAsDouble());
                    b.log("motorVoltage", motorVoltage.getValueAsDouble());
                    b.log("position", position.getValueAsDouble());
                    b.log("statorCurrent", statorCurrent.getValueAsDouble());
                    b.log("supplyCurrent", supplyCurrent.getValueAsDouble());
                    b.log("supplyVoltage", supplyVoltage.getValueAsDouble());
                    b.log("velocity", velocity.getValueAsDouble());
                };
            })
            .accept(backend);
    }
}
