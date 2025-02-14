package org.team340.lib.util.command;

import static org.team340.lib.util.command.GRRUtilities.*;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A {@link Subsystem} implementation.
 */
public abstract class GRRSubsystem implements Subsystem {

    public GRRSubsystem() {
        register();
    }

    /**
     * Creates a command builder that requires this subsystem.
     */
    public CommandBuilder commandBuilder() {
        return new CommandBuilder(this);
    }

    /**
     * Creates a command builder that requires this subsystem.
     * @param args The arguments of the command.
     */
    public CommandBuilder commandBuilder(String... args) {
        return new CommandBuilder(getMethodInfo(args), this);
    }
}
