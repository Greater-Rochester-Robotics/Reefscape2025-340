package org.team340.lib.util;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
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
import java.util.Optional;

/**
 * Utility class for interfacing with GRRDashboard.
 */
public final class GRRDashboard {

    public static final record AutoOption(
        String label,
        Command command,
        Optional<Pose2d> startingPose,
        RawPublisher pub
    ) {}

    private GRRDashboard() {
        throw new AssertionError("This is a utility class!");
    }

    private static final NetworkTable nt = NetworkTableInstance.getDefault().getTable("GRRDashboard");

    private static final String defaultAuto = "Do Nothing";
    private static final Map<String, AutoOption> autoOptions = new LinkedHashMap<>();
    private static final NetworkTable autoOptionsTable = nt.getSubTable("autos/options");
    private static final StringPublisher activeAutoPub = nt.getStringTopic("autos/active").publish();
    private static final StringSubscriber selectedAutoSub = nt.getStringTopic("autos/selected").subscribe(defaultAuto);

    private static AutoOption selectedAuto;

    static {
        selectedAuto = addAuto(defaultAuto, Commands.none(), List.of());
        activeAutoPub.setDefault(defaultAuto);
    }

    /**
     * Adds an auto to the dashboard.
     * @param routine The auto's {@link AutoRoutine}.
     */
    public static AutoOption addAuto(AutoRoutine routine) {
        return addAuto(routine, List.of());
    }

    /**
     * Adds an auto to the dashboard.
     * @param routine The auto's {@link AutoRoutine}.
     * @param trajectory The trajectory utilized by the auto.
     * @return The auto's label.
     */
    public static AutoOption addAuto(AutoRoutine routine, AutoTrajectory trajectory) {
        return addAuto(routine, List.of(trajectory));
    }

    /**
     * Adds an auto to the dashboard.
     * @param routine The auto's {@link AutoRoutine}.
     * @param trajectories A list of trajectories utilized by the auto.
     * @return The auto's label.
     */
    public static AutoOption addAuto(AutoRoutine routine, List<AutoTrajectory> trajectories) {
        return addAuto(routine.getName(), routine.cmd(), trajectories);
    }

    /**
     * Adds an auto to the dashboard.
     * @param label The label for the auto.
     * @param command The auto's command.
     * @param trajectories A list of trajectories utilized by the auto.
     * @return The auto's label.
     */
    public static AutoOption addAuto(String label, Command command, List<AutoTrajectory> trajectories) {
        List<Trajectory<SwerveSample>> loaded = new ArrayList<>();
        for (var trajectory : trajectories) {
            loaded.add(trajectory.getRawTrajectory());
        }

        Optional<Pose2d> startingPose = trajectories.isEmpty()
            ? Optional.empty()
            : trajectories.get(0).getInitialPose();

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

        AutoOption option = new AutoOption(label, command, startingPose, pub);
        autoOptions.put(label, option);
        pub.set(serialized);

        return option;
    }

    /**
     * Returns information about the selected auto.
     */
    public static AutoOption getSelectedAuto() {
        return selectedAuto;
    }

    /**
     * Returns a command that when scheduled will run the currently selected auto.
     */
    public static Command runSelectedAuto() {
        return Commands.deferredProxy(() -> selectedAuto.command()).withName("GRRDashboard.runSelectedAuto()");
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
                selectedAuto = option;
            }
        }
    }
}
