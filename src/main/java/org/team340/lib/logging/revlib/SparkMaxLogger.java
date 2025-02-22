package org.team340.lib.logging.revlib;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import org.team340.lib.logging.revlib.structs.SparkFaultsStruct;
import org.team340.lib.logging.revlib.structs.SparkWarningsStruct;

@CustomLoggerFor(SparkMax.class)
public class SparkMaxLogger extends ClassSpecificLogger<SparkMax> {

    public SparkMaxLogger() {
        super(SparkMax.class);
    }

    @Override
    public void update(EpilogueBackend backend, SparkMax sparkMax) {
        double appliedOutput = sparkMax.getAppliedOutput();

        backend.log("appliedOutput", appliedOutput);
        backend.log("appliedVoltage", appliedOutput * sparkMax.getBusVoltage());
        backend.log("motorTemperature", sparkMax.getMotorTemperature());
        backend.log("outputCurrent", sparkMax.getOutputCurrent());
        backend.log("position", sparkMax.getEncoder().getPosition());
        backend.log("velocity", sparkMax.getEncoder().getVelocity());
        backend.log("faults", sparkMax.getFaults().rawBits, SparkFaultsStruct.inst);
        backend.log("stickyFaults", sparkMax.getStickyFaults().rawBits, SparkFaultsStruct.inst);
        backend.log("warnings", sparkMax.getWarnings().rawBits, SparkWarningsStruct.inst);
        backend.log("stickyWarnings", sparkMax.getStickyWarnings().rawBits, SparkWarningsStruct.inst);
    }
}
