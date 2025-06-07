package org.team340.lib.logging;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.EpilogueConfiguration;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj.DriverStation;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.function.Consumer;
import org.team340.lib.util.Mutable;

//
// Do not fret if your IDE reports errors in this file!
//
// Epilogue is a generated class and may not have been seen yet by your
// editor, but your code will still build and function as normal.
//

/**
 * Utility for interfacing with the generated {@code Epilogue}
 * class. Keeps the occasional false negatives from the language
 * server in one file.
 */
public final class EpilogueProxy {

    private EpilogueProxy() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Configures Epilogue.
     * @param configurator A consumer that mutates the provided {@link EpilogueConfiguration}.
     */
    public static void configure(Consumer<EpilogueConfiguration> configurator) {
        Epilogue.configure(configurator);
    }

    /**
     * Gets the current epilogue configuration.
     */
    public static EpilogueConfiguration getConfig() {
        return Epilogue.getConfig();
    }

    /**
     * Gets the configured root backend.
     */
    public static EpilogueBackend getRootBackend() {
        return getConfig().backend.getNested(getConfig().root);
    }

    /**
     * Generates a logging function for the provided object via reflection. This
     * method should not be invoked periodically due to performance implications.
     * @param obj The object to generate the logger for.
     */
    @SuppressWarnings("unchecked")
    public static Consumer<EpilogueBackend> getLogger(Object obj) {
        Field[] fields = Epilogue.class.getDeclaredFields();
        Mutable<ClassSpecificLogger<Object>> objLogger = new Mutable<>(null);

        for (Field field : fields) {
            if (Modifier.isStatic(field.getModifiers())) {
                try {
                    Object value = field.get(obj);
                    if (value instanceof ClassSpecificLogger logger) {
                        Class<?> type = logger.getLoggedType();
                        if (type.equals(obj.getClass())) {
                            objLogger.value = logger;
                            break;
                        }
                    }
                } catch (Exception e) {}
            }
        }

        if (objLogger.value != null) {
            return backend -> objLogger.value.tryUpdate(backend, obj, Epilogue.getConfig().errorHandler);
        } else {
            DriverStation.reportWarning(
                "[EpilogueProxy] Unable to find logger for class \"" + obj.getClass().getSimpleName() + "\"",
                true
            );
            return backend -> {};
        }
    }
}
