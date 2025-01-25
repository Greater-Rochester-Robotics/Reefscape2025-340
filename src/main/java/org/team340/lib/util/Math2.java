package org.team340.lib.util;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.Optional;

/**
 * Math utilities.
 * We reinvented math. It's called Math 2.
 */
public final class Math2 {

    private Math2() {
        throw new AssertionError("This is a utility class!");
    }

    /** Shared maximum accuracy floating point. */
    public static final double kEpsilon = 1e-6;
    /** {@code PI/6} (30deg) */
    public static final double kSixthPi = Math.PI / 6.0;
    /** {@code PI/4} (45deg) */
    public static final double kQuarterPi = Math.PI / 4.0;
    /** {@code PI/3} (60deg) */
    public static final double kThirdPi = Math.PI / 3.0;
    /** {@code PI/2} (90deg) */
    public static final double kHalfPi = Math.PI / 2.0;
    /** {@code 2PI/3} (120deg) */
    public static final double kTwoThirdsPi = (2.0 * Math.PI) / 3.0;
    /** {@code 3PI/4} (135deg) */
    public static final double kThreeQuartersPi = (3.0 * Math.PI) / 4.0;
    /** {@code 5PI/6} (150deg) */
    public static final double kFiveSixthsPi = (5.0 * Math.PI) / 6.0;
    /** {@code PI*2} (360deg) */
    public static final double kTwoPi = Math.PI * 2.0;

    /**
     * Returns a random double from {@code 0.0} to {@code max}.
     * @param max The maximum value to return.
     */
    public static double random(double max) {
        return Math.random() * max;
    }

    /**
     * Returns a random double from {@code min} to {@code max}.
     * @param min The minimum value to return.
     * @param max The maximum value to return.
     */
    public static double random(double min, double max) {
        return (Math.random() * (max - min)) + min;
    }

    /**
     * Check if two optional values are equal within the accuracy of the default epsilon.
     * If one option is empty and the other is present, or both options are present and
     * unequal, {@code false} is returned. If both options empty, or both options are
     * present and equal, {@code true} is returned.
     * @param a The first value to compare.
     * @param b The second value to compare.
     * @return {@code true} if the values are equal.
     */
    public static boolean epsilonEquals(Optional<Double> a, Optional<Double> b) {
        return epsilonEquals(a, b, kEpsilon);
    }

    /**
     * Check if two optional values are equal within the accuracy of the default epsilon.
     * If one option is empty and the other is present, or both options are present and
     * unequal, {@code false} is returned. If both options empty, or both options are
     * present and equal, {@code true} is returned.
     * @param a The first value to compare.
     * @param b The second value to compare.
     * @param epsilon Epsilon value to compare with.
     * @return {@code true} if the values are equal.
     */
    public static boolean epsilonEquals(Optional<Double> a, Optional<Double> b, double epsilon) {
        if (a == b) return true;
        if (a.isEmpty() || b.isEmpty()) return a.isEmpty() && b.isEmpty();
        return epsilonEquals(a.get(), b.get(), epsilon);
    }

    /**
     * Check if two values are equal within the accuracy of the default epsilon.
     * @param a The first value to compare.
     * @param b The second value to compare.
     * @return {@code true} if the values are equal.
     */
    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, kEpsilon);
    }

    /**
     * Check if two values are equal within the accuracy of a provided epsilon.
     * @param a The first value to compare.
     * @param b The second value to compare.
     * @param epsilon Epsilon value to compare with.
     * @return {@code true} if the values are equal.
     */
    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    /**
     * Copies values from a source {@link Twist2d} object to another.
     * @param source The twist to copy from.
     * @param output The twist to copy into.
     * @return The output twist.
     */
    public static Twist2d copyInto(Twist2d source, Twist2d output) {
        output.dx = source.dx;
        output.dy = source.dy;
        output.dtheta = source.dtheta;
        return output;
    }

    /**
     * Copies values from a source {@link ChassisSpeeds} object to another.
     * @param source The speeds to copy from.
     * @param output The speeds to copy into.
     * @return The output speeds.
     */
    public static ChassisSpeeds copyInto(ChassisSpeeds source, ChassisSpeeds output) {
        output.vxMetersPerSecond = source.vxMetersPerSecond;
        output.vyMetersPerSecond = source.vyMetersPerSecond;
        output.omegaRadiansPerSecond = source.omegaRadiansPerSecond;
        return output;
    }

    /**
     * Copies values from a source {@link SwerveModulePosition} object to another.
     * @param source The swerve module position to copy from.
     * @param output The swerve module position to copy into.
     * @return The output position.
     */
    public static SwerveModulePosition copyInto(SwerveModulePosition source, SwerveModulePosition output) {
        output.distanceMeters = source.distanceMeters;
        output.angle = source.angle;
        return output;
    }

    /**
     * Copies values from a source {@link SwerveModuleState} object to another.
     * @param source The swerve module state to copy from.
     * @param output The swerve module state to copy into.
     * @return The output state.
     */
    public static SwerveModuleState copyInto(SwerveModuleState source, SwerveModuleState output) {
        output.speedMetersPerSecond = source.speedMetersPerSecond;
        output.angle = source.angle;
        return output;
    }

    /**
     * Zeroes a {@link ChassisSpeeds} object in place.
     * @param speeds The speeds to zero.
     * @return The provided speeds object.
     */
    public static ChassisSpeeds zero(ChassisSpeeds speeds) {
        speeds.vxMetersPerSecond = 0.0;
        speeds.vyMetersPerSecond = 0.0;
        speeds.omegaRadiansPerSecond = 0.0;
        return speeds;
    }

    /**
     * Zeroes a {@link Twist2d} object in place.
     * @param twist The twist to zero.
     * @return The provided twist object.
     */
    public static Twist2d zero(Twist2d twist) {
        twist.dx = 0.0;
        twist.dy = 0.0;
        twist.dtheta = 0.0;
        return twist;
    }
}
