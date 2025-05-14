package org.team340.lib.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.Supplier;
import org.team340.lib.util.Alliance;

/**
 * Utility class for flipping field locations based on the robot's alliance.
 */
public final class AllianceFlip {

    private final double fieldLength;
    private final double fieldWidth;
    private final FlipType flipType;

    /**
     * Create the alliance flipper.
     * @param fieldLength The length of the field, in meters.
     * @param fieldWidth The width of the field, in meters.
     * @param flipType The field's symmetry.
     */
    public AllianceFlip(double fieldLength, double fieldWidth, FlipType flipType) {
        this.fieldLength = fieldLength;
        this.fieldWidth = fieldWidth;
        this.flipType = flipType;
    }

    /**
     * Flips the provided translation to the opposite alliance.
     * @param translation The translation to flip.
     */
    public Translation2d flip(Translation2d translation) {
        return new Translation2d(
            flipType.x(translation.getX(), fieldLength),
            flipType.y(translation.getY(), fieldWidth)
        );
    }

    /**
     * Flips the provided rotation to the opposite alliance.
     * @param rotation The rotation to flip.
     */
    public Rotation2d flip(Rotation2d rotation) {
        return switch (flipType) {
            case MIRROR -> new Rotation2d(-rotation.getCos(), rotation.getSin());
            case ROTATE -> new Rotation2d(-rotation.getCos(), -rotation.getSin());
        };
    }

    /**
     * Flips the provided pose to the opposite alliance.
     * @param pose The pose to flip.
     */
    public Pose2d flip(Pose2d pose) {
        return new Pose2d(flip(pose.getTranslation()), flip(pose.getRotation()));
    }

    /**
     * Mirrors the provided translation across the width of the field.
     * @param translation The translation to mirror.
     */
    public Translation2d mirror(Translation2d translation) {
        return new Translation2d(translation.getX(), fieldWidth - translation.getY());
    }

    /**
     * Mirrors the provided rotation across the width of the field.
     * @param rotation The rotation to mirror.
     */
    public Rotation2d mirror(Rotation2d rotation) {
        return rotation.unaryMinus();
    }

    /**
     * Mirrors the provided pose across the width of the field.
     * @param pose The pose to mirror.
     */
    public Pose2d mirror(Pose2d pose) {
        return new Pose2d(mirror(pose.getTranslation()), mirror(pose.getRotation()));
    }

    /**
     * Creates a supplier that returns the provided {@link Pose2d},
     * flipped based on the robot's current alliance.
     * @param pose The blue origin relative pose.
     */
    public Supplier<Pose2d> flipped(Pose2d pose) {
        return flipped(pose, false);
    }

    /**
     * Creates a supplier that returns the provided {@link Pose2d},
     * flipped based on the robot's current alliance.
     * @param pose The blue origin relative pose.
     * @param mirrored If the pose should also be mirrored
     *                 across the width of the field.
     */
    public Supplier<Pose2d> flipped(Pose2d pose, boolean mirrored) {
        Pose2d blue = mirrored ? mirror(pose) : pose;
        Pose2d red = flip(pose);
        return () -> Alliance.isBlue() ? blue : red;
    }

    /**
     * Symmetry types for a field.
     */
    public static enum FlipType {
        MIRROR {
            @Override
            protected double x(double x, double fieldLength) {
                return fieldLength - x;
            }

            @Override
            protected double y(double y, double fieldWidth) {
                return y;
            }

            @Override
            protected double heading(double heading) {
                return Math.PI - heading;
            }
        },

        ROTATE {
            @Override
            protected double x(double x, double fieldLength) {
                return fieldLength - x;
            }

            @Override
            protected double y(double y, double fieldWidth) {
                return fieldWidth - y;
            }

            @Override
            protected double heading(double heading) {
                return Math.PI + heading;
            }
        };

        /**
         * Flips the X coordinate.
         * @param x The X coordinate to flip.
         * @param fieldLength The field's length.
         */
        protected abstract double x(double x, double fieldLength);

        /**
         * Flips the Y coordinate.
         * @param y The Y coordinate to flip.
         * @param fieldWidth the field's width.
         */
        protected abstract double y(double y, double fieldWidth);

        /**
         * Flips the heading.
         * @param heading The heading to flip.
         */
        protected abstract double heading(double heading);
    }
}
