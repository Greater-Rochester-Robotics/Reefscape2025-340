package org.team340.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.team340.lib.util.FieldFlip;
import org.team340.lib.util.FieldInfo;
import org.team340.lib.util.PAPFController.CircleObstacle;
import org.team340.lib.util.PAPFController.LateralObstacle;
import org.team340.lib.util.PAPFController.LineObstacle;
import org.team340.lib.util.PAPFController.LongitudinalObstacle;
import org.team340.lib.util.PAPFController.Obstacle;

/**
 * Field locations and utilities.
 */
public final class Field {

    public static final double PIPE_Y = 0.164;
    public static final double REEF_WALL_DIST = 0.781;

    public static final Translation2d REEF_BLUE = new Translation2d(4.489, FieldInfo.width() / 2.0);
    public static final Translation2d REEF_RED = FieldFlip.overLength(REEF_BLUE);

    public static final Pose2d STATION_STRAIGHT = new Pose2d(1.54, 0.71, Rotation2d.fromDegrees(54.0));
    public static final Pose2d STATION_FORWARDS = new Pose2d(1.54, 0.71, Rotation2d.fromDegrees(-36.0));
    public static final Pose2d STATION_BACKWARDS = new Pose2d(1.29, 0.89, Rotation2d.fromDegrees(144.0));

    public static final Pose2d AVOID_LEFT = new Pose2d(6.45, 0.9, Rotation2d.kPi);
    public static final Pose2d AVOID_RIGHT = FieldFlip.overWidth(AVOID_LEFT);

    private static final Translation2d CORAL_START = new Translation2d(0.0, 1.125);
    private static final Translation2d CORAL_END = new Translation2d(1.6, 0.0);

    public static final Obstacle[] OBSTACLES = {
        // Walls
        new LongitudinalObstacle(0.0, 0.1, true),
        new LongitudinalObstacle(FieldInfo.length(), 0.1, true),
        new LateralObstacle(0.0, 0.1, true),
        new LateralObstacle(FieldInfo.width(), 0.1, true),
        // Coral stations
        new LineObstacle(CORAL_START, CORAL_END, 0.1, true),
        new LineObstacle(FieldFlip.overWidth(CORAL_START), FieldFlip.overWidth(CORAL_END), 0.1, true),
        new LineObstacle(FieldFlip.overLength(CORAL_START), FieldFlip.overLength(CORAL_END), 0.1, true),
        new LineObstacle(FieldFlip.overDiagonal(CORAL_START), FieldFlip.overDiagonal(CORAL_END), 0.1, true),
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
