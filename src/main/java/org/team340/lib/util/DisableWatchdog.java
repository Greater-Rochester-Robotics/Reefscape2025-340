package org.team340.lib.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Watchdog;
import java.lang.reflect.Field;

public final class DisableWatchdog {

    private DisableWatchdog() {
        throw new AssertionError("This is a utility class!");
    }

    public static void in(Object obj, String fieldName) {
        try {
            Field field = null;
            Class<?> clazz = obj.getClass();

            while (field == null) {
                try {
                    field = clazz.getDeclaredField(fieldName);
                } catch (Exception e) {
                    clazz = clazz.getSuperclass();
                    if (clazz == null) throw new RuntimeException();
                }
            }

            field.setAccessible(true);
            Watchdog watchdog = (Watchdog) field.get(obj);
            watchdog.disable();
            watchdog.setTimeout(100000.0);
        } catch (Exception e) {
            DriverStation.reportWarning(
                "Unable to disable watchdog: Attempted with accessor \"" +
                obj.getClass().getSimpleName() +
                "." +
                fieldName +
                "\"",
                false
            );
        }
    }
}
