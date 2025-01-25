package org.team340.lib.util;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

/**
 * A simple pseudo call-graph profiler that publishes timings to NetworkTables.
 *
 * <p>Data is published to {@code /Profiling}, with CPU time utilized by the garbage
 * collector published at {@code /Profiling/GC}, and time spent publishing values to
 * NT at {@code /Profiling/Overhead}. All timings are recorded in  milliseconds,
 * as the duration of execution. The profiler records the time to execute blocks
 * of code, with support for nesting execution times which are reflected in an
 * expanding NT tree.
 *
 * <p>All invocations of {@link Profiler#start(String)} are expected to be
 * followed by a closing {@link Profiler#end()}. Additionally, it is expected
 * that a single {@link Profiler#start(String)}/{@link Profiler#end()} pair is
 * found at the highest level of the robot's code (i.e. {@code robotPeriodic}),
 * with no other "root" pairs. If either of these limitations are left unsatisfied,
 * an error will be printed to the Driver Station. Profiling across threads is also
 * not supported.
 */
public final class Profiler {

    private Profiler() {
        throw new AssertionError("This is a utility class!");
    }

    private static final NetworkTable nt = NetworkTableInstance.getDefault().getTable("Profiling");

    private static final Map<String, CallData> callGraph = new HashMap<>();
    private static final List<String> stack = new ArrayList<>();

    private static final List<GarbageCollectorMXBean> gcList = ManagementFactory.getGarbageCollectorMXBeans();
    private static final DoublePublisher overheadPub = nt.getDoubleTopic("overhead").publish();
    private static final DoublePublisher gcPub = nt.getDoubleTopic("gc").publish();

    private static String root = "";
    private static long lastGCTime = 0;

    /**
     * Determines if the profiler is currently running.
     * @return {@code true} if the profiler is running.
     */
    public static boolean isRunning() {
        return stack.size() > 0;
    }

    /**
     * Profiles a section of user code. Serves as a shorthand for calling
     * {@link Profiler#start(String)}, {user code}, {@link Profiler#end()}.
     * @param name The name of the call. Must be unique.
     * @param runnable The user code to be ran.
     */
    public static void run(String name, Runnable runnable) {
        start(name);
        runnable.run();
        end();
    }

    /**
     * Profiles a section of user code. Serves as a shorthand for calling
     * {@link Profiler#start(String)}, {user code}, {@link Profiler#end()}.
     * @param <T> The return type of the supplier.
     * @param name The name of the call. Must be unique.
     * @param runnable The user code to be ran.
     * @return The value returned from the supplier.
     */
    public static <T> T run(String name, Supplier<T> supplier) {
        start(name);
        T v = supplier.get();
        end();
        return v;
    }

    /**
     * Starts to profile a section of user code. This method should be invoked
     * before the user code to be profiled, with {@link Profiler#end()} expected
     * to be after. Additional {@link Profiler#start(String)}/{@link Profiler#end()}
     * pairs can be nested within the section of user code, and will appear as a
     * sub-topic in NetworkTables. Timings of nested pairs are still included in
     * the overall time recorded by the parent pair.
     * @param name The name of the call. Must be unique.
     */
    public static void start(String name) {
        if (stack.isEmpty()) {
            if (root.isEmpty()) {
                root = name;
            } else if (name != root) {
                DriverStation.reportError(
                    "[Profiler] Unexpected secondary root with name \"" +
                    name +
                    "\", expected primary root \"" +
                    root +
                    "\"",
                    true
                );
            }
        }

        stack.add(name);
        String fullName = String.join("/", stack);
        CallData call = callGraph.get(fullName);
        if (call == null) {
            call = new CallData(fullName);
            callGraph.put(fullName, call);
        }

        call.onStart();
    }

    /**
     * Ends profiling a section of code. This method should be invoked
     * after the user code to be profiled. Trailing calls to this method
     * will result in an error printed to the Driver Station.
     */
    public static void end() {
        CallData call = callGraph.get(String.join("/", stack));
        if (call != null) {
            call.onEnd();
            stack.remove(stack.size() - 1);

            if (stack.isEmpty()) {
                long start = RobotController.getFPGATime();
                var iter = callGraph.entrySet().iterator();
                while (iter.hasNext()) {
                    CallData entryCall = iter.next().getValue();
                    if (!entryCall.done) {
                        entryCall.close();
                        iter.remove();
                        continue;
                    }

                    entryCall.pubAndReset(start);
                }

                long gcSum = 0;
                for (var gc : gcList) {
                    long gcTime = gc.getCollectionTime();
                    if (gcTime != -1) gcSum += gcTime;
                }
                gcPub.set(gcSum - lastGCTime);
                lastGCTime = gcSum;

                overheadPub.set((RobotController.getFPGATime() - start) / 1000.0);
            }
        } else {
            DriverStation.reportError("[Profiler] Unexpected end() call", true);
        }
    }

    /**
     * Manages the data of a call.
     */
    private static final class CallData implements AutoCloseable {

        public final String fullName;
        public long time = -1;
        public boolean done = false;
        public DoublePublisher pub;

        public CallData(String fullName) {
            this.fullName = fullName;
        }

        /**
         * Should be invoked when a call starts.
         */
        public void onStart() {
            time = RobotController.getFPGATime();
        }

        /**
         * Should be invoked when a call ends.
         */
        public void onEnd() {
            time = RobotController.getFPGATime() - time;
            done = true;
        }

        /**
         * Publishes the last timing to NT and resets state.
         * @param timestamp The timestamp to publish the NT value to.
         */
        public void pubAndReset(long timestamp) {
            if (pub == null) pub = nt.getDoubleTopic(fullName).publish();
            pub.set(time / 1000.0, timestamp);
            time = -1;
            done = false;
        }

        @Override
        public void close() {
            pub.close();
        }
    }
}
