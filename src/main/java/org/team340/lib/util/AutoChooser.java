package org.team340.lib.util;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.HashMap;
import java.util.Map;

@Logged(strategy = Strategy.OPT_IN)
public final class AutoChooser {

    private static final String kDefault = "Do Nothing";

    private final StringArrayPublisher optionsPub;
    private final StringPublisher activePub;
    private final StringSubscriber selectedSub;

    private final Map<String, Command> options = new HashMap<>();

    private String activeName = kDefault;
    private Command activeCommand = Commands.none();

    public AutoChooser(String name) {
        NetworkTable nt = NetworkTableInstance.getDefault().getTable(NetworkTable.normalizeKey(name, true));

        nt.getStringTopic(".type").publish().set("String Chooser");
        nt.getBooleanTopic(".controllable").publish().set(true);
        nt.getStringTopic("default").publish().set(kDefault);

        optionsPub = nt.getStringArrayTopic("options").publish();
        activePub = nt.getStringTopic("active").publish();
        selectedSub = nt.getStringTopic("selected").subscribe("Do Nothing");

        add(activeName, activeCommand);
        activePub.set(activeName);
    }

    public void add(String name, Command command) {
        options.put(name, command);
        optionsPub.set(options.keySet().toArray(String[]::new));
    }

    public Command getSelected() {
        return activeCommand;
    }

    public Command runSelected() {
        return Commands.deferredProxy(() -> activeCommand);
    }

    public void update() {
        String selected = selectedSub.get();
        if (!selected.equals(activeName)) {
            activeName = options.containsKey(selected) ? selected : kDefault;
            activeCommand = options.get(activeName);
            activePub.set(activeName);
        }
    }
}
