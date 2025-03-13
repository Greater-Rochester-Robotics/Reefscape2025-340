package org.team340.lib.util;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.FloatEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.function.FloatConsumer;
import edu.wpi.first.wpilibj.event.EventLoop;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.IntConsumer;
import org.team340.lib.util.vendors.RevUtil;

/**
 * The Tunable class is used to construct tunable properties of the robot to be modified
 * via NetworkTables, as well as automatically displayed and edited in the dashboard.
 */
public final class Tunable {

    private Tunable() {
        throw new AssertionError("This is a utility class!");
    }

    private static final NetworkTable nt = NetworkTableInstance.getDefault().getTable("Tunables");
    private static final EventLoop pollChanges = new EventLoop();

    /**
     * Polls changes from NetworkTables. Must be called
     * periodically in order for this class to function.
     */
    public static void update() {
        pollChanges.poll();
    }

    /**
     * Creates a tunable boolean.
     * @param name The name for the tunable. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     */
    public static TunableBoolean booleanValue(String name, boolean defaultValue) {
        return booleanValue(name, defaultValue, null);
    }

    /**
     * Creates a tunable boolean.
     * @param name The name for the tunable. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     * @param onChange A consumer that is invoked when the value of the tunable is modified.
     */
    public static TunableBoolean booleanValue(String name, boolean defaultValue, BooleanConsumer onChange) {
        return new TunableBoolean(name, defaultValue, onChange);
    }

    /**
     * A tunable boolean value. Can be modified via NetworkTables.
     */
    @Logged(strategy = Strategy.OPT_IN)
    public static final class TunableBoolean implements AutoCloseable {

        private final BooleanEntry entry;
        private boolean value;

        private TunableBoolean(String name, boolean defaultValue, BooleanConsumer onChange) {
            entry = nt.getBooleanTopic(name).getEntry(defaultValue);
            entry.setDefault(defaultValue);
            value = defaultValue;

            pollChanges.bind(() -> {
                value = entry.get();
                if (onChange != null) {
                    boolean[] changes = entry.readQueueValues();
                    if (changes.length > 0) onChange.accept(changes[changes.length - 1]);
                }
            });
        }

        /**
         * Returns the value of the tunable.
         */
        public boolean value() {
            return value;
        }

        @Override
        public void close() {
            entry.close();
        }
    }

    /**
     * Creates a tunable integer.
     * @param name The name for the tunable. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     */
    public static TunableInteger intValue(String name, int defaultValue) {
        return intValue(name, defaultValue, null);
    }

    /**
     * Creates a tunable integer.
     * @param name The name for the tunable. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     * @param onChange A consumer that is invoked when the value of the tunable is modified.
     */
    public static TunableInteger intValue(String name, int defaultValue, IntConsumer onChange) {
        return new TunableInteger(name, defaultValue, onChange);
    }

    /**
     * A tunable integer value. Can be modified via NetworkTables.
     */
    @Logged(strategy = Strategy.OPT_IN)
    public static final class TunableInteger implements AutoCloseable {

        private final IntegerEntry entry;
        private int value;

        private TunableInteger(String name, int defaultValue, IntConsumer onChange) {
            entry = nt.getIntegerTopic(name).getEntry(defaultValue);
            entry.setDefault(defaultValue);
            value = defaultValue;

            pollChanges.bind(() -> {
                value = (int) entry.get();
                if (onChange != null) {
                    long[] changes = entry.readQueueValues();
                    if (changes.length > 0) onChange.accept((int) changes[changes.length - 1]);
                }
            });
        }

        /**
         * Returns the value of the tunable.
         */
        public int value() {
            return value;
        }

        @Override
        public void close() {
            entry.close();
        }
    }

    /**
     * Creates a tunable float.
     * @param name The name for the tunable. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     */
    public static TunableFloat floatValue(String name, float defaultValue) {
        return floatValue(name, defaultValue, null);
    }

    /**
     * Creates a tunable float.
     * @param name The name for the tunable. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     * @param onChange A consumer that is invoked when the value of the tunable is modified.
     */
    public static TunableFloat floatValue(String name, float defaultValue, FloatConsumer onChange) {
        return new TunableFloat(name, defaultValue, onChange);
    }

    /**
     * A tunable float value. Can be modified via NetworkTables.
     */
    @Logged(strategy = Strategy.OPT_IN)
    public static final class TunableFloat implements AutoCloseable {

        private final FloatEntry entry;
        private float value;

        private TunableFloat(String name, float defaultValue, FloatConsumer onChange) {
            entry = nt.getFloatTopic(name).getEntry(defaultValue);
            entry.setDefault(defaultValue);
            value = defaultValue;

            pollChanges.bind(() -> {
                value = entry.get();
                if (onChange != null) {
                    float[] changes = entry.readQueueValues();
                    if (changes.length > 0) onChange.accept(changes[changes.length - 1]);
                }
            });
        }

        /**
         * Returns the value of the tunable.
         */
        public float value() {
            return value;
        }

        @Override
        public void close() {
            entry.close();
        }
    }

    /**
     * Creates a tunable double.
     * @param name The name for the tunable. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     */
    public static TunableDouble doubleValue(String name, double defaultValue) {
        return doubleValue(name, defaultValue, null);
    }

    /**
     * Creates a tunable double.
     * @param name The name for the tunable. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     * @param onChange A consumer that is invoked when the value of the tunable is modified.
     */
    public static TunableDouble doubleValue(String name, double defaultValue, DoubleConsumer onChange) {
        return new TunableDouble(name, defaultValue, onChange);
    }

    /**
     * A tunable double value. Can be modified via NetworkTables.
     */
    @Logged(strategy = Strategy.OPT_IN)
    public static final class TunableDouble implements AutoCloseable {

        private final DoubleEntry entry;
        private double value;

        private TunableDouble(String name, double defaultValue, DoubleConsumer onChange) {
            entry = nt.getDoubleTopic(name).getEntry(defaultValue);
            entry.setDefault(defaultValue);
            value = defaultValue;

            pollChanges.bind(() -> {
                value = entry.get();
                if (onChange != null) {
                    double[] changes = entry.readQueueValues();
                    if (changes.length > 0) onChange.accept(changes[changes.length - 1]);
                }
            });
        }

        /**
         * Returns the value of the tunable.
         */
        public double value() {
            return value;
        }

        @Override
        public void close() {
            entry.close();
        }
    }

    /**
     * Creates a tunable string.
     * @param name The name for the tunable. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     */
    public static TunableString stringValue(String name, String defaultValue) {
        return stringValue(name, defaultValue, null);
    }

    /**
     * Creates a tunable string.
     * @param name The name for the tunable. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     * @param onChange A consumer that is invoked when the value of the tunable is modified.
     */
    public static TunableString stringValue(String name, String defaultValue, Consumer<String> onChange) {
        return new TunableString(name, defaultValue, onChange);
    }

    /**
     * A tunable string value. Can be modified via NetworkTables.
     */
    @Logged(strategy = Strategy.OPT_IN)
    public static final class TunableString implements AutoCloseable {

        private final StringEntry entry;
        private String value;

        private TunableString(String name, String defaultValue, Consumer<String> onChange) {
            entry = nt.getStringTopic(name).getEntry(defaultValue);
            entry.setDefault(defaultValue);
            value = defaultValue;

            pollChanges.bind(() -> {
                value = entry.get();
                if (onChange != null) {
                    String[] changes = entry.readQueueValues();
                    if (changes.length > 0) onChange.accept(changes[changes.length - 1]);
                }
            });
        }

        /**
         * Returns the value of the tunable.
         */
        public String value() {
            return value;
        }

        @Override
        public void close() {
            entry.close();
        }
    }

    /**
     * Wraps a WPILib {@link PIDController} to be tunable.
     * @param name The name for the tunable. Must be unique.
     * @param controller The PID controller.
     */
    public static void pidController(String name, PIDController controller) {
        doubleValue(name + "/kP", controller.getP(), v -> controller.setP(v));
        doubleValue(name + "/kI", controller.getI(), v -> controller.setI(v));
        doubleValue(name + "/kD", controller.getD(), v -> controller.setD(v));
        doubleValue(name + "/iZone", controller.getIZone(), v -> controller.setIZone(v));
    }

    /**
     * Wraps a WPILib {@link ProfiledPIDController} to be tunable.
     * @param name The name for the tunable. Must be unique.
     * @param controller The PID controller.
     */
    public static void pidController(String name, ProfiledPIDController controller) {
        doubleValue(name + "/kP", controller.getP(), v -> controller.setP(v));
        doubleValue(name + "/kI", controller.getI(), v -> controller.setI(v));
        doubleValue(name + "/kD", controller.getD(), v -> controller.setD(v));
        doubleValue(name + "/iZone", controller.getIZone(), v -> controller.setIZone(v));
        doubleValue(name + "/maxVelocity", controller.getConstraints().maxVelocity, v ->
            controller.setConstraints(new TrapezoidProfile.Constraints(v, controller.getConstraints().maxAcceleration))
        );
        doubleValue(name + "/maxAcceleration", controller.getConstraints().maxAcceleration, v ->
            controller.setConstraints(new TrapezoidProfile.Constraints(controller.getConstraints().maxVelocity, v))
        );
    }

    /**
     * Wraps a Spark's {@link SparkClosedLoopController} to be tunable.
     * @param name The name for the tunable. Must be unique.
     * @param spark The Spark to tune.
     */
    public static void pidController(String name, SparkMax spark) {
        pidController(name, spark, ClosedLoopSlot.kSlot0);
    }

    /**
     * Wraps a Spark's {@link SparkClosedLoopController} to be tunable.
     * @param name The name for the tunable. Must be unique.
     * @param spark The Spark to tune.
     * @param slot The config slot to use.
     */
    public static void pidController(String name, SparkMax spark, ClosedLoopSlot slot) {
        var config = spark.configAccessor.closedLoop;

        doubleValue(name + "/kP", config.getP(slot), v -> {
            var newConfig = new SparkMaxConfig();
            newConfig.closedLoop.p(v, slot);
            RevUtil.configEphemeral(spark, newConfig);
        });
        doubleValue(name + "/kI", config.getI(slot), v -> {
            var newConfig = new SparkMaxConfig();
            newConfig.closedLoop.i(v, slot);
            RevUtil.configEphemeral(spark, newConfig);
        });
        doubleValue(name + "/kD", config.getD(slot), v -> {
            var newConfig = new SparkMaxConfig();
            newConfig.closedLoop.d(v, slot);
            RevUtil.configEphemeral(spark, newConfig);
        });
        doubleValue(name + "/kV", config.getFF(slot), v -> {
            var newConfig = new SparkMaxConfig();
            newConfig.closedLoop.velocityFF(v, slot);
            RevUtil.configEphemeral(spark, newConfig);
        });
        doubleValue(name + "/iZone", config.getIZone(slot), v -> {
            var newConfig = new SparkMaxConfig();
            newConfig.closedLoop.iZone(v, slot);
            RevUtil.configEphemeral(spark, newConfig);
        });
        doubleValue(name + "/dFilter", config.getDFilter(slot), v -> {
            var newConfig = new SparkMaxConfig();
            newConfig.closedLoop.dFilter(v, slot);
            RevUtil.configEphemeral(spark, newConfig);
        });
        doubleValue(name + "/minOutput", config.getMinOutput(slot), v -> {
            var newConfig = new SparkMaxConfig();
            newConfig.closedLoop.minOutput(v, slot);
            RevUtil.configEphemeral(spark, newConfig);
        });
        doubleValue(name + "/maxOutput", config.getMaxOutput(slot), v -> {
            var newConfig = new SparkMaxConfig();
            newConfig.closedLoop.maxOutput(v, slot);
            RevUtil.configEphemeral(spark, newConfig);
        });
    }

    /**
     * Wraps a Spark's {@link SparkClosedLoopController} to be tunable.
     * @param name The name for the tunable. Must be unique.
     * @param spark The Spark to tune.
     */
    public static void pidController(String name, SparkFlex spark) {
        pidController(name, spark, ClosedLoopSlot.kSlot0);
    }

    /**
     * Wraps a Spark's {@link SparkClosedLoopController} to be tunable.
     * @param name The name for the tunable. Must be unique.
     * @param spark The Spark to tune.
     * @param slot The config slot to use.
     */
    public static void pidController(String name, SparkFlex spark, ClosedLoopSlot slot) {
        var config = spark.configAccessor.closedLoop;

        doubleValue(name + "/kP", config.getP(slot), v -> {
            var newConfig = new SparkFlexConfig();
            newConfig.closedLoop.p(v, slot);
            RevUtil.configEphemeral(spark, newConfig);
        });
        doubleValue(name + "/kI", config.getI(slot), v -> {
            var newConfig = new SparkFlexConfig();
            newConfig.closedLoop.i(v, slot);
            RevUtil.configEphemeral(spark, newConfig);
        });
        doubleValue(name + "/kD", config.getD(slot), v -> {
            var newConfig = new SparkFlexConfig();
            newConfig.closedLoop.d(v, slot);
            RevUtil.configEphemeral(spark, newConfig);
        });
        doubleValue(name + "/kV", config.getFF(slot), v -> {
            var newConfig = new SparkFlexConfig();
            newConfig.closedLoop.velocityFF(v, slot);
            RevUtil.configEphemeral(spark, newConfig);
        });
        doubleValue(name + "/iZone", config.getIZone(slot), v -> {
            var newConfig = new SparkFlexConfig();
            newConfig.closedLoop.iZone(v, slot);
            RevUtil.configEphemeral(spark, newConfig);
        });
        doubleValue(name + "/dFilter", config.getDFilter(slot), v -> {
            var newConfig = new SparkFlexConfig();
            newConfig.closedLoop.dFilter(v, slot);
            RevUtil.configEphemeral(spark, newConfig);
        });
        doubleValue(name + "/minOutput", config.getMinOutput(slot), v -> {
            var newConfig = new SparkFlexConfig();
            newConfig.closedLoop.minOutput(v, slot);
            RevUtil.configEphemeral(spark, newConfig);
        });
        doubleValue(name + "/maxOutput", config.getMaxOutput(slot), v -> {
            var newConfig = new SparkFlexConfig();
            newConfig.closedLoop.maxOutput(v, slot);
            RevUtil.configEphemeral(spark, newConfig);
        });
    }

    /**
     * Enables tuning a {@link TalonFX}'s PID config.
     * Note that this only applies to the slot 0 config.
     * @param name The name for the tunable. Must be unique.
     * @param talonFX The TalonFX to tune.
     */
    public static void pidController(String name, TalonFX talonFX) {
        Slot0Configs config = new Slot0Configs();
        talonFX.getConfigurator().refresh(config);

        doubleValue(name + "/kP", config.kP, v -> {
            talonFX.getConfigurator().refresh(config);
            config.kP = v;
            talonFX.getConfigurator().apply(config);
        });
        doubleValue(name + "/kI", config.kI, v -> {
            talonFX.getConfigurator().refresh(config);
            config.kI = v;
            talonFX.getConfigurator().apply(config);
        });
        doubleValue(name + "/kD", config.kD, v -> {
            talonFX.getConfigurator().refresh(config);
            config.kD = v;
            talonFX.getConfigurator().apply(config);
        });
        doubleValue(name + "/kS", config.kS, v -> {
            talonFX.getConfigurator().refresh(config);
            config.kS = v;
            talonFX.getConfigurator().apply(config);
        });
        doubleValue(name + "/kV", config.kV, v -> {
            talonFX.getConfigurator().refresh(config);
            config.kV = v;
            talonFX.getConfigurator().apply(config);
        });
        doubleValue(name + "/kA", config.kA, v -> {
            talonFX.getConfigurator().refresh(config);
            config.kA = v;
            talonFX.getConfigurator().apply(config);
        });
        doubleValue(name + "/kG", config.kG, v -> {
            talonFX.getConfigurator().refresh(config);
            config.kG = v;
            talonFX.getConfigurator().apply(config);
        });
    }

    /**
     * Enables tuning a {@link SparkMax}'s MAXMotion config.
     * @param name The name for the tunable. Must be unique.
     * @param spark The Spark to tune.
     */
    public static void motionProfile(String name, SparkMax spark) {
        motionProfile(name, spark, ClosedLoopSlot.kSlot0);
    }

    /**
     * Enables tuning a {@link SparkMax}'s MAXMotion config.
     * @param name The name for the tunable. Must be unique.
     * @param spark The Spark to tune.
     * @param slot The config slot to use.
     */
    public static void motionProfile(String name, SparkMax spark, ClosedLoopSlot slot) {
        var config = spark.configAccessor.closedLoop.maxMotion;

        doubleValue(name + "/velocity", config.getMaxVelocity(slot), v -> {
            var newConfig = new SparkMaxConfig();
            newConfig.closedLoop.maxMotion.maxVelocity(v, slot);
            RevUtil.configEphemeral(spark, newConfig);
        });
        doubleValue(name + "/acceleration", config.getMaxAcceleration(slot), v -> {
            var newConfig = new SparkMaxConfig();
            newConfig.closedLoop.maxMotion.maxAcceleration(v, slot);
            RevUtil.configEphemeral(spark, newConfig);
        });
        doubleValue(name + "/allowedClosedLoopError", config.getAllowedClosedLoopError(slot), v -> {
            var newConfig = new SparkMaxConfig();
            newConfig.closedLoop.maxMotion.allowedClosedLoopError(v, slot);
            RevUtil.configEphemeral(spark, newConfig);
        });
    }

    /**
     * Enables tuning a {@link SparkFlex}'s MAXMotion config.
     * @param name The name for the tunable. Must be unique.
     * @param spark The Spark to tune.
     */
    public static void motionProfile(String name, SparkFlex spark) {
        motionProfile(name, spark, ClosedLoopSlot.kSlot0);
    }

    /**
     * Enables tuning a {@link SparkFlex}'s MAXMotion config.
     * @param name The name for the tunable. Must be unique.
     * @param spark The Spark to tune.
     * @param slot The config slot to use.
     */
    public static void motionProfile(String name, SparkFlex spark, ClosedLoopSlot slot) {
        var config = spark.configAccessor.closedLoop.maxMotion;

        doubleValue(name + "/velocity", config.getMaxVelocity(slot), v -> {
            var newConfig = new SparkFlexConfig();
            newConfig.closedLoop.maxMotion.maxVelocity(v, slot);
            RevUtil.configEphemeral(spark, newConfig);
        });
        doubleValue(name + "/acceleration", config.getMaxAcceleration(slot), v -> {
            var newConfig = new SparkFlexConfig();
            newConfig.closedLoop.maxMotion.maxAcceleration(v, slot);
            RevUtil.configEphemeral(spark, newConfig);
        });
        doubleValue(name + "/allowedClosedLoopError", config.getAllowedClosedLoopError(slot), v -> {
            var newConfig = new SparkFlexConfig();
            newConfig.closedLoop.maxMotion.allowedClosedLoopError(v, slot);
            RevUtil.configEphemeral(spark, newConfig);
        });
    }

    /**
     * Enables tuning a {@link TalonFX}'s motion magic config.
     * @param The name for the tunable. Must be unique.
     * @param talonFX The TalonFX to tune.
     */
    public static void motionProfile(String name, TalonFX talonFX) {
        MotionMagicConfigs config = new MotionMagicConfigs();
        talonFX.getConfigurator().refresh(config);

        doubleValue(name + "/velocity", config.MotionMagicCruiseVelocity, v -> {
            talonFX.getConfigurator().refresh(config);
            config.MotionMagicCruiseVelocity = v;
            talonFX.getConfigurator().apply(config);
        });
        doubleValue(name + "/acceleration", config.MotionMagicAcceleration, v -> {
            talonFX.getConfigurator().refresh(config);
            config.MotionMagicAcceleration = v;
            talonFX.getConfigurator().apply(config);
        });
        doubleValue(name + "/jerk", config.MotionMagicJerk, v -> {
            talonFX.getConfigurator().refresh(config);
            config.MotionMagicJerk = v;
            talonFX.getConfigurator().apply(config);
        });
        doubleValue(name + "/expoKv", config.MotionMagicExpo_kV, v -> {
            talonFX.getConfigurator().refresh(config);
            config.MotionMagicExpo_kV = v;
            talonFX.getConfigurator().apply(config);
        });
        doubleValue(name + "/expoKa", config.MotionMagicExpo_kA, v -> {
            talonFX.getConfigurator().refresh(config);
            config.MotionMagicExpo_kA = v;
            talonFX.getConfigurator().apply(config);
        });
    }
}
