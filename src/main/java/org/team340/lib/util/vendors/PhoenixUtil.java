package org.team340.lib.util.vendors;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

/**
 * Utilities for the CTRE Phoenix 6 API.
 */
public final class PhoenixUtil {

    private PhoenixUtil() {
        throw new AssertionError("This is a utility class!");
    }

    /**
     * Runs a Phoenix API call and checks for errors. Will
     * try up to 3 times if the target API call fails.
     * @param name The name of the API call.
     * @param target The target call to run.
     * @return {@code true} if success ({@link StatusCode#isOK()}), {@code false} otherwise.
     */
    public static boolean run(String name, Supplier<StatusCode> target) {
        return run(name, target, 3);
    }

    /**
     * Runs a Phoenix API call and checks for errors.
     * @param name The name of the API call.
     * @param target The target call to run.
     * @param maxTries The number of times to try the call before failing. {@code 1} only runs the call once.
     * @return {@code true} if success ({@link StatusCode#isOK()}), {@code false} otherwise.
     */
    public static boolean run(String name, Supplier<StatusCode> target, int maxTries) {
        String results = "";
        for (int i = 0; i < maxTries; i++) {
            StatusCode result = target.get();
            if (result.isOK()) return true;
            results += (results.isEmpty() ? "" : ", ") + result.name();
        }

        DriverStation.reportError("[PhoenixUtil] " + name + "\": " + results, false);
        return false;
    }
}
