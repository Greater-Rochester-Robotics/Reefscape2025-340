package org.team340.lib.logging.revlib;

import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(SparkClosedLoopController.class)
public class SparkClosedLoopControllerLogger extends ClassSpecificLogger<SparkClosedLoopController> {

    public SparkClosedLoopControllerLogger() {
        super(SparkClosedLoopController.class);
    }

    @Override
    public void update(EpilogueBackend backend, SparkClosedLoopController pidController) {
        // No-op
    }
}
