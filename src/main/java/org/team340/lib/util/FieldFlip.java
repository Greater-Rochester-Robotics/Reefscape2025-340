package org.team340.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.Supplier;

/**
 * Utility class for flipping field locations across lines of symmetry.
 * By default, {@link FieldInfo} is utilized to retrieve field dimensions.
 */
public final class FieldFlip {

    private FieldFlip() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    private static double length = FieldInfo.length();
    private static double width = FieldInfo.width();

    /**
     * Overrides the default field dimensions. This method
     * should be called before all other user code.
     * @param length The field's length, in meters.
     * @param width The field's width, in meters.
     */
    public static void setFieldDimensions(double length, double width) {
        FieldFlip.length = length;
        FieldFlip.width = width;
    }

    /**
     * Flips the provided {@link Translation2d} over the
     * length of the field (i.e. hamburger style).
     * @param translation The translation to flip.
     */
    public static Translation2d overLength(Translation2d translation) {
        return new Translation2d(length - translation.getX(), translation.getY());
    }

    /**
     * Flips the provided {@link Rotation2d} over the
     * length of the field (i.e. hamburger style).
     * @param rotation The rotation to flip.
     */
    public static Rotation2d overLength(Rotation2d rotation) {
        return new Rotation2d(-rotation.getCos(), rotation.getSin());
    }

    /**
     * Flips the provided {@link Pose2d} over the
     * length of the field (i.e. hamburger style).
     * @param pose The pose to flip.
     */
    public static Pose2d overLength(Pose2d pose) {
        return new Pose2d(overLength(pose.getTranslation()), overLength(pose.getRotation()));
    }

    /**
     * Flips the provided {@link Translation2d} over the
     * width of the field (i.e. hotdog style).
     * @param translation The translation to flip.
     */
    public static Translation2d overWidth(Translation2d translation) {
        return new Translation2d(translation.getX(), width - translation.getY());
    }

    /**
     * Flips the provided {@link Rotation2d} over the
     * width of the field (i.e. hotdog style).
     * @param rotation The rotation to flip.
     */
    public static Rotation2d overWidth(Rotation2d rotation) {
        return rotation.unaryMinus();
    }

    /**
     * Flips the provided {@link Pose2d} over the
     * width of the field (i.e. hotdog style).
     * @param pose The pose to flip.
     */
    public static Pose2d overWidth(Pose2d pose) {
        return new Pose2d(overWidth(pose.getTranslation()), overWidth(pose.getRotation()));
    }

    /**
     * Flips the provided {@link Translation2d} over the field's
     * diagonal (i.e. rotated 180deg around the field's center).
     * @param translation The translation to flip.
     */
    public static Translation2d overDiagonal(Translation2d translation) {
        return new Translation2d(length - translation.getX(), width - translation.getY());
    }

    /**
     * Flips the provided {@link Rotation2d} over the field's
     * diagonal (i.e. rotated 180deg around the field's center).
     * @param rotation The rotation to flip.
     */
    public static Rotation2d overDiagonal(Rotation2d rotation) {
        return new Rotation2d(-rotation.getCos(), -rotation.getSin());
    }

    /**
     * Flips the provided {@link Pose2d} over the field's
     * diagonal (i.e. rotated 180deg around the field's center).
     * @param pose The pose to flip.
     */
    public static Pose2d overDiagonal(Pose2d pose) {
        return new Pose2d(overDiagonal(pose.getTranslation()), overDiagonal(pose.getRotation()));
    }

    /**
     * Creates a supplier that returns the provided blue origin relative
     * {@link Pose2d}, optionally flipped over the length of the field
     * (i.e. hamburger style) if the robot is on the red alliance.
     * @param pose The blue origin relative pose.
     */
    public static Supplier<Pose2d> allianceLength(Pose2d pose) {
        Pose2d flipped = overLength(pose);
        return () -> Alliance.isBlue() ? pose : flipped;
    }

    /**
     * Creates a supplier that returns the provided blue origin relative
     * {@link Pose2d}, optionally flipped over the field's diagonal
     * (i.e. rotated 180deg around the field's center) if the robot
     * is on the red alliance.
     * @param pose The blue origin relative pose.
     */
    public static Supplier<Pose2d> allianceDiagonal(Pose2d pose) {
        Pose2d flipped = overDiagonal(pose);
        return () -> Alliance.isBlue() ? pose : flipped;
    }
}
