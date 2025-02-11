package org.team340.lib.logging.phoenix;

import static edu.wpi.first.util.struct.StructGenerator.genEnum;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.signals.S1StateValue;
import com.ctre.phoenix6.signals.S2StateValue;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.util.struct.Struct;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

@CustomLoggerFor(CANdi.class)
public class CANdiLogger extends ClassSpecificLogger<CANdi> {

    private static final Map<CANdi, Consumer<EpilogueBackend>> cache = new HashMap<>();
    private static final Struct<S1StateValue> s1StateStruct = genEnum(S1StateValue.class);
    private static final Struct<S2StateValue> s2StateStruct = genEnum(S2StateValue.class);

    public CANdiLogger() {
        super(CANdi.class);
    }

    @Override
    public void update(EpilogueBackend backend, CANdi candi) {
        cache
            .computeIfAbsent(candi, key -> {
                var pwm1Position = candi.getPWM1Position(false);
                var pwm1Velocity = candi.getPWM1Velocity(false);
                var pwm2Position = candi.getPWM2Position(false);
                var pwm2Velocity = candi.getPWM2Velocity(false);
                var quadraturePosition = candi.getQuadraturePosition(false);
                var quadratureVelocity = candi.getQuadratureVelocity(false);
                var s1Closed = candi.getS1Closed(false);
                var s1State = candi.getS1State(false);
                var s2Closed = candi.getS2Closed(false);
                var s2State = candi.getS2State(false);

                BaseStatusSignal[] signals = {
                    pwm1Position,
                    pwm1Velocity,
                    pwm2Position,
                    pwm2Velocity,
                    quadraturePosition,
                    quadratureVelocity,
                    s1Closed,
                    s1State,
                    s2Closed,
                    s2State
                };

                return b -> {
                    BaseStatusSignal.refreshAll(signals);
                    b.log("pwm1Position", pwm1Position.getValueAsDouble());
                    b.log("pwm1Velocity", pwm1Velocity.getValueAsDouble());
                    b.log("pwm2Position", pwm2Position.getValueAsDouble());
                    b.log("pwm2Velocity", pwm2Velocity.getValueAsDouble());
                    b.log("quadraturePosition", quadraturePosition.getValueAsDouble());
                    b.log("quadratureVelocity", quadratureVelocity.getValueAsDouble());
                    b.log("s1Closed", s1Closed.getValue());
                    b.log("s1State", s1State.getValue(), s1StateStruct);
                    b.log("s2Closed", s2Closed.getValue());
                    b.log("s2State", s2State.getValue(), s2StateStruct);
                };
            })
            .accept(backend);
    }
}
