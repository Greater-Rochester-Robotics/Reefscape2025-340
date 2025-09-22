package org.team340.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import org.team340.lib.math.FieldInfo;
import org.team340.lib.math.PAPFController.CircleObstacle;
import org.team340.lib.math.PAPFController.LateralObstacle;
import org.team340.lib.math.PAPFController.LineObstacle;
import org.team340.lib.math.PAPFController.LongitudinalObstacle;
import org.team340.lib.math.PAPFController.Obstacle;
import org.team340.lib.math.geometry.ExtPose;
import org.team340.lib.math.geometry.ExtTranslation;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;

/**
 * Field locations and utilities.
 */
public final class Field {

    public static final double pipeY = 0.164;
    public static final double reefWallDist = 0.781;

    public static final ExtTranslation reef = new ExtTranslation(4.489, FieldInfo.width() / 2.0);

    public static final ExtPose loadStraight = new ExtPose(1.54, 0.71, Rotation2d.fromDegrees(54.0));
    public static final ExtPose loadForwards = new ExtPose(1.54, 0.71, Rotation2d.fromDegrees(-36.0));
    public static final ExtPose loadBackwards = new ExtPose(1.29, 0.89, Rotation2d.fromDegrees(144.0));

    public static final ExtPose avoid = new ExtPose(6.45, 0.9, Rotation2d.k180deg);

    private static final double robotRadius = 0.253;
    private static final ExtTranslation coralStart = new ExtTranslation(robotRadius, 1.125 + robotRadius);
    private static final ExtTranslation coralEnd = new ExtTranslation(1.6 + robotRadius, robotRadius);

    public static final Obstacle[] obstacles = {
        // Walls
        new LongitudinalObstacle(robotRadius, 1.1, 2.0),
        new LongitudinalObstacle(FieldInfo.length() - robotRadius, 1.1, 2.0),
        new LateralObstacle(robotRadius, 1.1, 2.0),
        new LateralObstacle(FieldInfo.width() - robotRadius, 1.1, 2.0),
        // Coral stations
        new LineObstacle(coralStart.getBlue(), coralEnd.getBlue(), 1.1, 2.0),
        new LineObstacle(coralStart.getBlue(true), coralEnd.getBlue(true), 1.1, 2.0),
        new LineObstacle(coralStart.getRed(), coralEnd.getRed(), 1.1, 2.0),
        new LineObstacle(coralStart.getRed(true), coralEnd.getRed(true), 1.1, 2.0),
        // Reef
        new CircleObstacle(reef.getBlue(), 0.83, 4.0, 1.5),
        new CircleObstacle(reef.getRed(), 0.83, 4.0, 1.5)
    };

    public static enum ReefLocation {
        A(0.0, true, false),
        B(0.0, false, false),
        C(60.0, true, false),
        D(60.0, false, false),
        E(120.0, true, true),
        F(120.0, false, true),
        G(180.0, true, true),
        H(180.0, false, true),
        I(-120.0, true, true),
        J(-120.0, false, true),
        K(-60.0, true, false),
        L(-60.0, false, false);

        public final Rotation2d side;
        public final boolean left;
        public final boolean back;

        private ReefLocation(double degrees, boolean left, boolean back) {
            this.side = Rotation2d.fromDegrees(degrees);
            this.left = left;
            this.back = back;
        }
    }

    static {
        TunableTable tunables = Tunables.getNested("field");
        tunables.add("loadStraight", loadStraight);
        tunables.add("loadForwards", loadForwards);
        tunables.add("loadBackwards", loadBackwards);
        tunables.add("avoid", avoid);
    }
}
