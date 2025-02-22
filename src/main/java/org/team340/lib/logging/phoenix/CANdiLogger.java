package org.team340.lib.logging.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANdi;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

@CustomLoggerFor(CANdi.class)
public class CANdiLogger extends ClassSpecificLogger<CANdi> {

    private static final Map<CANdi, Consumer<EpilogueBackend>> cache = new HashMap<>();

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
                var s1Closed = candi.getS1Closed(false);
                var s2Closed = candi.getS2Closed(false);

                BaseStatusSignal[] signals = {
                    pwm1Position,
                    pwm1Velocity,
                    pwm2Position,
                    pwm2Velocity,
                    s1Closed,
                    s2Closed
                };

                return b -> {
                    BaseStatusSignal.refreshAll(signals);
                    b.log("pwm1Position", pwm1Position.getValueAsDouble());
                    b.log("pwm1Velocity", pwm1Velocity.getValueAsDouble());
                    b.log("pwm2Position", pwm2Position.getValueAsDouble());
                    b.log("pwm2Velocity", pwm2Velocity.getValueAsDouble());
                    b.log("s1Closed", s1Closed.getValue());
                    b.log("s2Closed", s2Closed.getValue());
                };
            })
            .accept(backend);
    }
}
