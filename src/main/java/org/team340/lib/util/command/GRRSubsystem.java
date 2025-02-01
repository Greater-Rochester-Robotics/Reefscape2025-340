package org.team340.lib.util.command;

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
     * @param name The name of the command.
     */
    public CommandBuilder commandBuilder(String name) {
        return new CommandBuilder(name, this);
    }

    /**
     * Creates a commands name in the form subsystemName.methodName(args).
     * @param args The args to use as strings.
     * @return The name of the command.
     */
    protected String getMethodInfo(String... args) {
        return (
            getName() +
            "." +
            Thread.currentThread().getStackTrace()[2].getMethodName() +
            "(" +
            String.join(",", args) +
            ")"
        );
    }

    /**
     * Gets the name of the class enclosing the specified object.
     * Use {@code getEnclosingClassName(new Object() {})} to get the name of the enclosing class.
     * @param obj The object to use.
     * @return The name of the objects enclosing class.
     */
    protected static String getEnclosingClassName(Object obj) {
        return obj.getClass().getEnclosingClass().getSimpleName();
    }
}
