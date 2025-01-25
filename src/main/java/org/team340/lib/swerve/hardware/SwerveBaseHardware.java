package org.team340.lib.swerve.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import java.util.List;
import org.team340.lib.swerve.SwerveAPI;

interface SwerveBaseHardware extends AutoCloseable {
    /**
     * The CAN frame period in seconds to use for frames containing data
     * utilized for telemetry that is not necessarily required for swerve
     * to function.
     */
    static final double kTelemetryCANPeriod = 0.2;

    /**
     * Returns the device's underlying API.
     */
    public abstract Object getAPI();

    /**
     * Logs the device via Epilogue.
     * @param logger The logger to log data to.
     * @param errorHandler The handler to use if logging raised an exception.
     */
    public abstract void log(EpilogueBackend backend, ErrorHandler errorHandler);

    /**
     * Returns all Phoenix status signals in use by the hardware. Phoenix
     * hardware should <i>not</i> invoke {@code .refresh()} on their status
     * signals in their implementations. This method is required for the
     * odometry thread to register signals to be refreshed automatically.
     * Because signals are not thread safe, all returned signals should
     * also be cloned in their initialization as to not interfere with
     * telemetry, which is invoked on the main thread. The exception to
     * this rule is IMU pitch and roll values, as they are only measured
     * synchronously when calling {@link SwerveAPI#refresh()}.
     */
    public default List<BaseStatusSignal> getSignals() {
        return List.of();
    }

    /**
     * If the device has encountered an error while reading inputs.
     */
    public default boolean readError() {
        return false;
    }
}
