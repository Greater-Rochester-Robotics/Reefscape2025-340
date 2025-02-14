package org.team340.lib.util.command;

public class GRRUtilities {

    // This is the index of the calling method in the array returned by getStackTrace().
    private static int kCallingMethodIndex = 2;

    public static String getSubsystemName() {
        return Thread.currentThread().getStackTrace()[kCallingMethodIndex].getClassName();
    }

    /**
     * Creates a commands name in the form subsystemName.methodName(args).
     * @param args The args to use as strings.
     * @return The name of the command.
     */
    public static String getMethodInfo(String... args) {
        StackTraceElement element = Thread.currentThread().getStackTrace()[kCallingMethodIndex];
        return (element.getClassName() + "." + element.getMethodName() + "(" + String.join(",", args) + ")");
    }

    /**
     * Gets the name of the provided enum for use in tunable names.
     * @param innerEnum The enum to get the name of.
     */
    public static String getEnumName(Enum<?> innerEnum) {
        return (
            innerEnum.getClass().getEnclosingClass().getSimpleName() +
            "/" +
            innerEnum.getClass().getSimpleName() +
            "/" +
            innerEnum.name()
        );
    }
}
