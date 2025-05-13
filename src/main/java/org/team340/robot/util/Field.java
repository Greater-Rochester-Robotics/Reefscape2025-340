package org.team340.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.Supplier;
import org.team340.lib.util.Alliance;
import org.team340.lib.util.PAPFController.CircleObstacle;
import org.team340.lib.util.PAPFController.LateralObstacle;
import org.team340.lib.util.PAPFController.LineObstacle;
import org.team340.lib.util.PAPFController.LongitudinalObstacle;
import org.team340.lib.util.PAPFController.Obstacle;

/**
 * Field locations and utilities.
 */
public final class Field {

    public static final double kLength = 17.548;
    public static final double kWidth = 8.052;
    private static final Flipper kFlipper = Flipper.kRotate;

    public static final double kPipeOffsetY = 0.164;
    public static final double kReefCenterToWallDistance = 0.781;

    public static final Translation2d kReefCenterBlue = new Translation2d(4.489, kWidth / 2.0);
    public static final Translation2d kReefCenterRed = flip(kReefCenterBlue);

    public static final Translation2d kStation = new Translation2d(1.53, 0.64);
    public static final Pose2d kStationForwards = new Pose2d(kStation, Rotation2d.fromDegrees(-36.0));
    public static final Pose2d kStationBackwards = new Pose2d(kStation, Rotation2d.fromDegrees(144.0));
    public static final Pose2d kAvoidLocation = new Pose2d(6.45, 0.9, Rotation2d.kPi);

    private static final Translation2d kCoralStart = new Translation2d(0.0, 1.125);
    private static final Translation2d kCoralEnd = new Translation2d(1.6, 0.0);

    public static final Obstacle[] kObstacles = {
        // Walls
        new LongitudinalObstacle(0.0, 0.1, true),
        new LongitudinalObstacle(kLength, 0.1, true),
        new LateralObstacle(0.0, 0.1, true),
        new LateralObstacle(kWidth, 0.1, true),
        // Coral stations
        new LineObstacle(kCoralStart, kCoralEnd, 0.1, true),
        new LineObstacle(mirror(kCoralStart), mirror(kCoralEnd), 0.1, true),
        new LineObstacle(flip(kCoralStart), flip(kCoralEnd), 0.1, true),
        new LineObstacle(flip(mirror(kCoralStart)), flip(mirror(kCoralEnd)), 0.1, true),
        // Reef
        new CircleObstacle(kReefCenterBlue, 0.83, 1.0),
        new CircleObstacle(kReefCenterRed, 0.83, 1.0)
    };

    public static enum ReefLocation {
        A(Rotation2d.fromDegrees(0.0), true, false),
        B(Rotation2d.fromDegrees(0.0), false, false),
        C(Rotation2d.fromDegrees(60.0), true, false),
        D(Rotation2d.fromDegrees(60.0), false, false),
        E(Rotation2d.fromDegrees(120.0), true, true),
        F(Rotation2d.fromDegrees(120.0), false, true),
        G(Rotation2d.fromDegrees(180.0), true, true),
        H(Rotation2d.fromDegrees(180.0), false, true),
        I(Rotation2d.fromDegrees(-120.0), true, true),
        J(Rotation2d.fromDegrees(-120.0), false, true),
        K(Rotation2d.fromDegrees(-60.0), true, false),
        L(Rotation2d.fromDegrees(-60.0), false, false);

        public final Rotation2d side;
        public final boolean left;
        public final boolean back;

        private ReefLocation(Rotation2d side, boolean left, boolean back) {
            this.side = side;
            this.left = left;
            this.back = back;
        }
    }

    /**
     * Flips the provided translation to the opposite alliance.
     * @param translation The translation to flip.
     */
    public static Translation2d flip(Translation2d translation) {
        return new Translation2d(kFlipper.x(translation.getX()), kFlipper.y(translation.getY()));
    }

    /**
     * Flips the provided rotation to the opposite alliance.
     * @param rotation The rotation to flip.
     */
    public static Rotation2d flip(Rotation2d rotation) {
        return switch (kFlipper) {
            case kMirror -> new Rotation2d(-rotation.getCos(), rotation.getSin());
            case kRotate -> new Rotation2d(-rotation.getCos(), -rotation.getSin());
        };
    }

    /**
     * Flips the provided pose to the opposite alliance.
     * @param pose The pose to flip.
     */
    public static Pose2d flip(Pose2d pose) {
        return new Pose2d(flip(pose.getTranslation()), flip(pose.getRotation()));
    }

    /**
     * Mirrors the provided translation across the width of the field.
     * @param translation The translation to mirror.
     */
    public static Translation2d mirror(Translation2d translation) {
        return new Translation2d(translation.getX(), kWidth - translation.getY());
    }

    /**
     * Mirrors the provided rotation across the width of the field.
     * @param rotation The rotation to mirror.
     */
    public static Rotation2d mirror(Rotation2d rotation) {
        return rotation.unaryMinus();
    }

    /**
     * Mirrors the provided pose across the width of the field.
     * @param pose The pose to mirror.
     */
    public static Pose2d mirror(Pose2d pose) {
        return new Pose2d(mirror(pose.getTranslation()), mirror(pose.getRotation()));
    }

    /**
     * Creates a supplier that returns the provided {@link Pose2d},
     * flipped based on the robot's current alliance.
     * @param pose The blue origin relative pose.
     */
    public static Supplier<Pose2d> flipped(Pose2d pose) {
        return flipped(pose, false);
    }

    /**
     * Creates a supplier that returns the provided {@link Pose2d},
     * flipped based on the robot's current alliance.
     * @param pose The blue origin relative pose.
     * @param mirrored If the pose should also be mirrored
     *                 across the width of the field.
     */
    public static Supplier<Pose2d> flipped(Pose2d pose, boolean mirrored) {
        Pose2d blue = mirrored ? mirror(pose) : pose;
        Pose2d red = flip(pose);
        return () -> Alliance.isBlue() ? blue : red;
    }

    private static enum Flipper {
        kMirror {
            public double x(double x) {
                return kLength - x;
            }

            public double y(double y) {
                return y;
            }

            public double heading(double heading) {
                return Math.PI - heading;
            }
        },

        kRotate {
            public double x(double x) {
                return kLength - x;
            }

            public double y(double y) {
                return kWidth - y;
            }

            public double heading(double heading) {
                return Math.PI + heading;
            }
        };

        /**
         * Flips the X coordinate.
         * @param x The X coordinate to flip.
         */
        public abstract double x(double x);

        /**
         * Flips the Y coordinate.
         * @param y The Y coordinate to flip.
         */
        public abstract double y(double y);

        /**
         * Flips the heading.
         * @param heading The heading to flip.
         */
        public abstract double heading(double heading);
    }
}
