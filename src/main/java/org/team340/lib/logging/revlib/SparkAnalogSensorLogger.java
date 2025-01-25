package org.team340.lib.logging.revlib;

import com.revrobotics.spark.SparkAnalogSensor;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(SparkAnalogSensor.class)
public class SparkAnalogSensorLogger extends ClassSpecificLogger<SparkAnalogSensor> {

    public SparkAnalogSensorLogger() {
        super(SparkAnalogSensor.class);
    }

    @Override
    public void update(EpilogueBackend backend, SparkAnalogSensor analogSensor) {
        backend.log("position", analogSensor.getPosition());
        backend.log("velocity", analogSensor.getVelocity());
        backend.log("voltage", analogSensor.getVoltage());
    }
}
