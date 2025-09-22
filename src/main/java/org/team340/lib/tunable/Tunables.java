package org.team340.lib.tunable;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.FloatEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.function.FloatConsumer;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.IntConsumer;
import org.team340.lib.logging.LoggedRobot;

/**
 * The Tunables class is used to construct tunable properties
 * of the robot to be modified live via NetworkTables.
 */
public final class Tunables {

    /**
     * Base interface for tunable objects.
     */
    @FunctionalInterface
    public static interface Tunable {
        /**
         * Initializes the object to be tuned. This method is
         * called when adding the object to a {@link TunableTable}.
         * @param table The table the object is being added to.
         */
        public void initTunable(TunableTable table);
    }

    /**
     * Interface for wrapping objects to be tunable. Instances of this interface should
     * be passed to {@link Tunables#registerHandler(Class, TunableHandler)} in order for
     * objects of the handler's type to be processed.
     */
    @FunctionalInterface
    public static interface TunableHandler<T> {
        /**
         * This method should implement the setup for an object to be tunable.
         * @param table The table to object is being added to.
         * @param obj The object to make tunable.
         */
        void init(TunableTable table, T obj);
    }

    private static final NetworkTable nt = NetworkTableInstance.getDefault().getTable("/Tunables");
    private static final Set<Runnable> pollChanges = new LinkedHashSet<>();

    private static final Map<String, TunableTable> tables = new HashMap<>();
    private static final Map<Class<?>, TunableHandler<?>> handlers = new HashMap<>();

    static {
        ThirdParty.registerAll();
    }

    /**
     * Polls changes from NetworkTables. Must be called periodically in order for
     * Tunables to function (this is already done if utilizing {@link LoggedRobot}).
     */
    public static void update() {
        pollChanges.forEach(Runnable::run);
    }

    /**
     * Gets a table that can be used to add nested
     * tunable values under a specified path.
     * @param name The name of the table.
     */
    public static TunableTable getNested(String name) {
        name = NetworkTable.normalizeKey(name, false);
        if (!name.endsWith("/")) name += "/";

        return tables.computeIfAbsent(name, key -> new TunableTable(key));
    }

    /**
     * Registers a handler for tuning objects.
     * @param <T> The object's class.
     * @param clazz The object's class.
     * @param handler The handler for the object.
     */
    public static <T> void registerHandler(Class<T> clazz, TunableHandler<T> handler) {
        handlers.put(clazz, handler);
    }

    /**
     * Adds an object to be tuned. The specified object must implement {@link Tunable},
     * or have a registered {@link TunableHandler} in order for it to be added.
     * @param <T> The object's class.
     * @param name The name for the tunable. Must be unique.
     * @param obj The object to be tuned.
     * @return The provided object.
     */
    public static <T> T add(String name, T obj) {
        if (obj instanceof Tunable tunable) {
            tunable.initTunable(getNested(name));
        } else {
            boolean initialized = false;
            for (var entry : handlers.entrySet()) {
                if (entry.getKey().isInstance(obj)) {
                    @SuppressWarnings("unchecked")
                    var handler = (TunableHandler<T>) entry.getValue();
                    handler.init(getNested(name), obj);
                    initialized = true;
                    break;
                }
            }

            if (!initialized) {
                DriverStation.reportWarning(
                    "[Tunables] Unable to find TunableHandler for type \"" + obj.getClass().getSimpleName() + "\"",
                    true
                );
            }
        }

        return obj;
    }

    /**
     * Creates a tunable boolean value.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     */
    public static TunableBoolean value(String name, boolean defaultValue) {
        return value(name, defaultValue, null);
    }

    /**
     * Creates a tunable boolean value.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     * @param onChange A consumer that is invoked when the value of the tunable is modified.
     */
    public static TunableBoolean value(String name, boolean defaultValue, BooleanConsumer onChange) {
        return new TunableBoolean(name, defaultValue, onChange);
    }

    /**
     * A tunable boolean value. Can be modified via NetworkTables.
     */
    public static final class TunableBoolean implements AutoCloseable {

        private final BooleanEntry entry;
        private boolean value;

        private TunableBoolean(String name, boolean defaultValue, BooleanConsumer onChange) {
            entry = nt.getBooleanTopic(name).getEntry(defaultValue);
            entry.setDefault(defaultValue);
            value = defaultValue;

            pollChanges.add(() -> {
                value = entry.get();
                if (onChange != null) {
                    for (boolean value : entry.readQueueValues()) {
                        onChange.accept(value);
                    }
                }
            });
        }

        /**
         * Returns the value of the tunable.
         */
        public boolean get() {
            return value;
        }

        /**
         * Sets the value of the tunable.
         * @param value The new value.
         */
        public void set(boolean value) {
            this.value = value;
            entry.set(value);
        }

        @Override
        public void close() {
            entry.close();
        }
    }

    /**
     * Creates a tunable integer value.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     */
    public static TunableInteger value(String name, int defaultValue) {
        return value(name, defaultValue, (IntConsumer) null);
    }

    /**
     * Creates a tunable integer value.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     * @param onChange A consumer that is invoked when the value of the tunable is modified.
     */
    public static TunableInteger value(String name, int defaultValue, IntConsumer onChange) {
        return new TunableInteger(name, defaultValue, onChange);
    }

    /**
     * A tunable integer value. Can be modified via NetworkTables.
     */
    public static final class TunableInteger implements AutoCloseable {

        private final IntegerEntry entry;
        private int value;

        private TunableInteger(String name, int defaultValue, IntConsumer onChange) {
            entry = nt.getIntegerTopic(name).getEntry(defaultValue);
            entry.setDefault(defaultValue);
            value = defaultValue;

            pollChanges.add(() -> {
                value = (int) entry.get();
                if (onChange != null) {
                    for (long value : entry.readQueueValues()) {
                        onChange.accept((int) value);
                    }
                }
            });
        }

        /**
         * Returns the value of the tunable.
         */
        public int get() {
            return value;
        }

        /**
         * Sets the value of the tunable.
         * @param value The new value.
         */
        public void set(int value) {
            this.value = value;
            entry.set(value);
        }

        @Override
        public void close() {
            entry.close();
        }
    }

    /**
     * Creates a tunable float value.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     */
    public static TunableFloat value(String name, float defaultValue) {
        return value(name, defaultValue, (FloatConsumer) null);
    }

    /**
     * Creates a tunable float value.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     * @param onChange A consumer that is invoked when the value of the tunable is modified.
     */
    public static TunableFloat value(String name, float defaultValue, FloatConsumer onChange) {
        return new TunableFloat(name, defaultValue, onChange);
    }

    /**
     * A tunable float value. Can be modified via NetworkTables.
     */
    public static final class TunableFloat implements AutoCloseable {

        private final FloatEntry entry;
        private float value;

        private TunableFloat(String name, float defaultValue, FloatConsumer onChange) {
            entry = nt.getFloatTopic(name).getEntry(defaultValue);
            entry.setDefault(defaultValue);
            value = defaultValue;

            pollChanges.add(() -> {
                value = entry.get();
                if (onChange != null) {
                    for (float value : entry.readQueueValues()) {
                        onChange.accept(value);
                    }
                }
            });
        }

        /**
         * Returns the value of the tunable.
         */
        public float get() {
            return value;
        }

        /**
         * Sets the value of the tunable.
         * @param value The new value.
         */
        public void set(float value) {
            this.value = value;
            entry.set(value);
        }

        @Override
        public void close() {
            entry.close();
        }
    }

    /**
     * Creates a tunable double value.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     */
    public static TunableDouble value(String name, double defaultValue) {
        return value(name, defaultValue, (DoubleConsumer) null);
    }

    /**
     * Creates a tunable double value.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     * @param onChange A consumer that is invoked when the value of the tunable is modified.
     */
    public static TunableDouble value(String name, double defaultValue, DoubleConsumer onChange) {
        return new TunableDouble(name, defaultValue, onChange);
    }

    /**
     * A tunable double value. Can be modified via NetworkTables.
     */
    public static final class TunableDouble implements AutoCloseable {

        private final DoubleEntry entry;
        private double value;

        private TunableDouble(String name, double defaultValue, DoubleConsumer onChange) {
            entry = nt.getDoubleTopic(name).getEntry(defaultValue);
            entry.setDefault(defaultValue);
            value = defaultValue;

            pollChanges.add(() -> {
                value = entry.get();
                if (onChange != null) {
                    for (double value : entry.readQueueValues()) {
                        onChange.accept(value);
                    }
                }
            });
        }

        /**
         * Returns the value of the tunable.
         */
        public double get() {
            return value;
        }

        /**
         * Sets the value of the tunable.
         * @param value The new value.
         */
        public void set(double value) {
            this.value = value;
            entry.set(value);
        }

        @Override
        public void close() {
            entry.close();
        }
    }

    /**
     * Creates a tunable string value.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     */
    public static TunableString value(String name, String defaultValue) {
        return value(name, defaultValue, null);
    }

    /**
     * Creates a tunable string value.
     * @param name The name for the tunable value. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     * @param onChange A consumer that is invoked when the value of the tunable is modified.
     */
    public static TunableString value(String name, String defaultValue, Consumer<String> onChange) {
        return new TunableString(name, defaultValue, onChange);
    }

    /**
     * A tunable string value. Can be modified via NetworkTables.
     */
    public static final class TunableString implements AutoCloseable {

        private final StringEntry entry;
        private String value;

        private TunableString(String name, String defaultValue, Consumer<String> onChange) {
            entry = nt.getStringTopic(name).getEntry(defaultValue);
            entry.setDefault(defaultValue);
            value = defaultValue;

            pollChanges.add(() -> {
                value = entry.get();
                if (onChange != null) {
                    for (String value : entry.readQueueValues()) {
                        onChange.accept(value);
                    }
                }
            });
        }

        /**
         * Returns the value of the tunable.
         */
        public String get() {
            return value;
        }

        /**
         * Sets the value of the tunable.
         * @param value The new value.
         */
        public void set(String value) {
            this.value = value;
            entry.set(value);
        }

        @Override
        public void close() {
            entry.close();
        }
    }
}
