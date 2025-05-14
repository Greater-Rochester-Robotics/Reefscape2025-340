package org.team340.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.team340.lib.math.AllianceFlip;
import org.team340.lib.math.AllianceFlip.FlipType;
import org.team340.lib.math.PAPFController.CircleObstacle;
import org.team340.lib.math.PAPFController.LateralObstacle;
import org.team340.lib.math.PAPFController.LineObstacle;
import org.team340.lib.math.PAPFController.LongitudinalObstacle;
import org.team340.lib.math.PAPFController.Obstacle;

/**
 * Field locations and utilities.
 */
public final class Field {

    public static final double LENGTH = 17.548;
    public static final double WIDTH = 8.052;
    public static final AllianceFlip FLIPPER = new AllianceFlip(LENGTH, WIDTH, FlipType.ROTATE);

    public static final double PIPE_Y = 0.164;
    public static final double REEF_WALL_DIST = 0.781;

    public static final Translation2d REEF_BLUE = new Translation2d(4.489, WIDTH / 2.0);
    public static final Translation2d REEF_RED = FLIPPER.flip(REEF_BLUE);

    public static final Translation2d STATION = new Translation2d(1.53, 0.64);
    public static final Pose2d STATION_FORWARDS = new Pose2d(STATION, Rotation2d.fromDegrees(-36.0));
    public static final Pose2d STATION_BACKWARDS = new Pose2d(STATION, Rotation2d.fromDegrees(144.0));
    public static final Pose2d AVOID_LOCATION = new Pose2d(6.45, 0.9, Rotation2d.kPi);

    private static final Translation2d CORAL_START = new Translation2d(0.0, 1.125);
    private static final Translation2d CORAL_END = new Translation2d(1.6, 0.0);

    public static final Obstacle[] OBSTACLES = {
        // Walls
        new LongitudinalObstacle(0.0, 0.1, true),
        new LongitudinalObstacle(LENGTH, 0.1, true),
        new LateralObstacle(0.0, 0.1, true),
        new LateralObstacle(WIDTH, 0.1, true),
        // Coral stations
        new LineObstacle(CORAL_START, CORAL_END, 0.1, true),
        new LineObstacle(FLIPPER.mirror(CORAL_START), FLIPPER.mirror(CORAL_END), 0.1, true),
        new LineObstacle(FLIPPER.flip(CORAL_START), FLIPPER.flip(CORAL_END), 0.1, true),
        new LineObstacle(FLIPPER.flip(FLIPPER.mirror(CORAL_START)), FLIPPER.flip(FLIPPER.mirror(CORAL_END)), 0.1, true),
        // Reef
        new CircleObstacle(REEF_BLUE, 0.83, 1.0),
        new CircleObstacle(REEF_RED, 0.83, 1.0)
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
}
