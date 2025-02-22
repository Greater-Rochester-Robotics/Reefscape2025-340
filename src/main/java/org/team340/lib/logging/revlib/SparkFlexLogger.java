package org.team340.lib.logging.revlib;

import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import org.team340.lib.logging.revlib.structs.SparkFaultsStruct;
import org.team340.lib.logging.revlib.structs.SparkWarningsStruct;

@CustomLoggerFor(SparkFlex.class)
public class SparkFlexLogger extends ClassSpecificLogger<SparkFlex> {

    public SparkFlexLogger() {
        super(SparkFlex.class);
    }

    @Override
    public void update(EpilogueBackend backend, SparkFlex sparkFlex) {
        double appliedOutput = sparkFlex.getAppliedOutput();

        backend.log("appliedOutput", appliedOutput);
        backend.log("appliedVoltage", appliedOutput * sparkFlex.getBusVoltage());
        backend.log("motorTemperature", sparkFlex.getMotorTemperature());
        backend.log("outputCurrent", sparkFlex.getOutputCurrent());
        backend.log("position", sparkFlex.getEncoder().getPosition());
        backend.log("velocity", sparkFlex.getEncoder().getVelocity());
        backend.log("faults", sparkFlex.getFaults().rawBits, SparkFaultsStruct.inst);
        backend.log("stickyFaults", sparkFlex.getStickyFaults().rawBits, SparkFaultsStruct.inst);
        backend.log("warnings", sparkFlex.getWarnings().rawBits, SparkWarningsStruct.inst);
        backend.log("stickyWarnings", sparkFlex.getStickyWarnings().rawBits, SparkWarningsStruct.inst);
    }
}
