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
     * @param args The arguments of the command.
     */
    public CommandBuilder commandBuilder(String... args) {
        return new CommandBuilder(getMethodInfo(args), this);
    }

    // This is the index of the calling method in the array returned by getStackTrace().
    private static int kCallingMethodIndex = 2;

    protected static String getSubsystemName() {
        return Thread.currentThread().getStackTrace()[kCallingMethodIndex].getClassName();
    }

    /**
     * Creates a commands name in the form subsystemName.methodName(args).
     * @param args The args to use as strings.
     * @return The name of the command.
     */
    protected static String getMethodInfo(String... args) {
        StackTraceElement element = Thread.currentThread().getStackTrace()[kCallingMethodIndex];
        return (element.getClassName() + "." + element.getMethodName() + "(" + String.join(",", args) + ")");
    }

    /**
     * Gets the name of the provided enum for use in tunable names.
     * @param innerEnum The enum to get the name of.
     */
    protected static String getEnumName(Enum<?> innerEnum) {
        return (
            innerEnum.getClass().getEnclosingClass().getSimpleName() +
            "/" +
            innerEnum.getClass().getSimpleName() +
            "/" +
            innerEnum.name()
        );
    }
}
