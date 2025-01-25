package org.team340.lib.util;

import choreo.Choreo.TrajectoryCache;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.RawPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Utility class for interfacing with GRRDashboard.
 */
public final class GRRDashboard {

    private GRRDashboard() {
        throw new AssertionError("This is a utility class!");
    }

    private static final NetworkTable nt = NetworkTableInstance.getDefault().getTable("GRRDashboard");

    private static final String defaultAuto = "Do Nothing";
    private static final Map<String, Pair<Command, RawPublisher>> autoOptions = new LinkedHashMap<>();
    private static final NetworkTable autoOptionsTable = nt.getSubTable("autos/options");
    private static final StringPublisher activeAutoPub = nt.getStringTopic("autos/active").publish();
    private static final StringSubscriber selectedAutoSub = nt.getStringTopic("autos/selected").subscribe(defaultAuto);

    private static TrajectoryCache trajectoryCache = new TrajectoryCache();
    private static Command selectedAuto = Commands.none();

    static {
        addAuto(defaultAuto, selectedAuto);
        activeAutoPub.setDefault(defaultAuto);
    }

    /**
     * Sets the Choreo {@link TrajectoryCache} in use. Utilized for loading trajectories.
     * @param cache The trajectory cache in use by the robot.
     */
    public static void setTrajectoryCache(TrajectoryCache cache) {
        trajectoryCache = cache;
    }

    /**
     * Adds an auto to the dashboard.
     * @param label The label for the auto.
     * @param command The auto's command.
     * @return The auto's label.
     */
    public static String addAuto(String label, Command command) {
        return addAuto(label, List.of(), command);
    }

    /**
     * Adds an auto to the dashboard.
     * @param label The label for the auto.
     * @param trajectory The name of the trajectory utilized by the auto.
     * @param command The auto's command.
     * @return The auto's label.
     */
    public static String addAuto(String label, String trajectory, Command command) {
        return addAuto(label, List.of(trajectory), command);
    }

    /**
     * Adds an auto to the dashboard.
     * @param label The label for the auto.
     * @param trajectories A list of the names of trajectories utilized by the auto.
     * @param command The auto's command.
     * @return The auto's label.
     */
    @SuppressWarnings("unchecked")
    public static String addAuto(String label, List<String> trajectories, Command command) {
        List<Trajectory<SwerveSample>> loaded = new ArrayList<>();
        for (String name : trajectories) {
            var trajectory = trajectoryCache.loadTrajectory(name);
            loaded.add(
                trajectory.isPresent()
                    ? (Trajectory<SwerveSample>) trajectory.get()
                    : new Trajectory<SwerveSample>("", List.of(), List.of(), List.of())
            );
        }

        int size = loaded.stream().mapToInt(t -> t.samples().size() * 16).sum();
        ByteBuffer serialized = ByteBuffer.allocate(size + 4);

        double t = 0.0;
        for (int i = 0; i < loaded.size(); i++) {
            var trajectory = loaded.get(i);
            for (SwerveSample sample : trajectory.samples()) {
                serialized
                    .putFloat((float) sample.x)
                    .putFloat((float) sample.y)
                    .putFloat((float) sample.heading)
                    .putFloat((float) (sample.t + t));
            }

            var last = trajectory.getFinalSample(false);
            if (last.isPresent()) t += last.get().t;
        }

        serialized.putFloat((float) t);
        var pub = autoOptionsTable.getRawTopic(label).publish("raw");
        autoOptions.put(label, Pair.of(command, pub));
        pub.set(serialized);

        return label;
    }

    /**
     * Gets the command of the currently selected auto. Note that this method
     * is not intended for use with triggers or command compositions, as the
     * returned command will not update when the selection changes. Use
     * {@link GRRDashboard#runSelectedAuto()} instead.
     */
    public static Command getSelectedAuto() {
        return selectedAuto;
    }

    /**
     * Returns a command that when scheduled will run the currently selected auto.
     */
    public static Command runSelectedAuto() {
        return Commands.defer(() -> selectedAuto.asProxy(), Set.of()).withName("GRRDashboard.runSelectedAuto()");
    }

    /**
     * Syncs data with the dashboard. Must be called
     * periodically in order for this class to function.
     */
    public static void update() {
        String[] selections = selectedAutoSub.readQueueValues();
        if (selections.length > 0) {
            String selection = selections[selections.length - 1];
            var option = autoOptions.get(selection);
            if (option != null) {
                activeAutoPub.set(selection);
                selectedAuto = option.getFirst();
            }
        }
    }
}
