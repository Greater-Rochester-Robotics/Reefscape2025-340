package org.team340.lib.logging.revlib;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(SparkAbsoluteEncoder.class)
public class SparkAbsoluteEncoderLogger extends ClassSpecificLogger<SparkAbsoluteEncoder> {

    public SparkAbsoluteEncoderLogger() {
        super(SparkAbsoluteEncoder.class);
    }

    @Override
    public void update(EpilogueBackend backend, SparkAbsoluteEncoder absoluteEncoder) {
        backend.log("position", absoluteEncoder.getPosition());
        backend.log("velocity", absoluteEncoder.getVelocity());
    }
}
