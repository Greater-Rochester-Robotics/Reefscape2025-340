package org.team340.lib.util;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;

/**
 * Similar to {@link SendableChooser}, this class implements a selector over
 * NetworkTables for choosing a command to run during the autonomous period.
 *
 * <p>For this class to function, you must either bind it to a command
 * scheduler using {@link AutoChooser#bind(CommandScheduler)}, or call
 * {@link AutoChooser#update()} periodically.
 */
@Logged(strategy = Strategy.OPT_IN)
public final class AutoChooser {

    private static final String DEFAULT = "Do Nothing";

    private final StringArrayPublisher optionsPub;
    private final StringPublisher activePub;
    private final StringSubscriber selectedSub;

    private final Map<String, Command> options = new HashMap<>();

    private String activeName = DEFAULT;
    private Command activeCommand = Commands.none();

    /**
     * Create the auto chooser.
     * @param name The name for the chooser in NetworkTables.
     */
    public AutoChooser(String name) {
        NetworkTable nt = NetworkTableInstance.getDefault().getTable(name);

        nt.getStringTopic(".type").publish().set("String Chooser");
        nt.getBooleanTopic(".controllable").publish().set(true);
        nt.getStringTopic("default").publish().set(DEFAULT);

        optionsPub = nt.getStringArrayTopic("options").publish();
        activePub = nt.getStringTopic("active").publish();
        selectedSub = nt.getStringTopic("selected").subscribe("Do Nothing");

        add(activeName, activeCommand);
        activePub.set(activeName);
    }

    /**
     * Returns the command of the currently selected auto.
     */
    public Command getSelected() {
        return activeCommand;
    }

    /**
     * Add an option to the chooser.
     * @param name The name of the option. Must be unique.
     * @param command The option's command.
     */
    public void add(String name, Command command) {
        options.put(name, command);
        optionsPub.set(options.keySet().toArray(String[]::new));
    }

    /**
     * Binds the auto chooser to the default button loop of the provided command
     * scheduler. This will automatically update the chooser, as well as schedule
     * the selected command during the autonomous period.
     * @param scheduler The command scheduler to bind to.
     */
    public void bind(CommandScheduler scheduler) {
        scheduler.getDefaultButtonLoop().bind(this::update);
        new Trigger(scheduler.getDefaultButtonLoop(), DriverStation::isAutonomousEnabled).whileTrue(
            Commands.deferredProxy(() -> activeCommand)
        );
    }

    /**
     * Updates the auto chooser. You do not need to call this method
     * when using {@link AutoChooser#bind(CommandScheduler)}.
     */
    public void update() {
        String selected = selectedSub.get();
        if (!selected.equals(activeName)) {
            activeName = options.containsKey(selected) ? selected : DEFAULT;
            activeCommand = options.get(activeName);
            activePub.set(activeName);
        }
    }
}
