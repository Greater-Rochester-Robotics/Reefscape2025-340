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
                var closedLoopReference = talonFXS.getClosedLoopReference(false);
                var deviceTemp = talonFXS.getDeviceTemp(false);
                var motorVoltage = talonFXS.getMotorVoltage(false);
                var position = talonFXS.getPosition(false);
                var statorCurrent = talonFXS.getStatorCurrent(false);
                var velocity = talonFXS.getVelocity(false);

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
