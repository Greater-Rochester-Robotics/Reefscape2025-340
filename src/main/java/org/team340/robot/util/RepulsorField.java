package org.team340.robot.util;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

// TODO Finish comments and probably optimize the math

/**
 * A pathfinder designed to avoid defined obstacles on an FRC field while targeting a final
 * goal pose. Originally utilized by 1736 Robot Casserole, this implementation is based on
 * 6995 NOMAD and 167 Children of the Corn's iterations of the original algorithm.
 */
@Logged(strategy = Strategy.OPT_IN)
public final class RepulsorField {

    public final record RepulsorSample(Pose2d nextTarget, double vx, double vy) {}

    private final double period;
    private final List<Obstacle> obstacles;

    @Logged
    private Pose2d target = Pose2d.kZero;

    public RepulsorField(double period, List<Obstacle> obstacles) {
        this.period = period;
        this.obstacles = obstacles;
    }

    public Pose2d getTarget() {
        return target;
    }

    public void setTarget(Pose2d newTarget) {
        target = newTarget;
    }

    public RepulsorSample sampleField(Translation2d location, double maxVelocity, double slowdownRange) {
        var error = location.minus(target.getTranslation());
        var netForce = getTargetForce(location).plus(getObstacleForce(location));

        double stepSize;
        if (error.getNorm() < slowdownRange) {
            stepSize = MathUtil.interpolate(0.0, maxVelocity * period, error.getNorm() / slowdownRange);
        } else {
            stepSize = maxVelocity * period;
        }

        var step = new Translation2d(stepSize, netForce.getAngle());
        return new RepulsorSample(
            new Pose2d(location.plus(step), target.getRotation()),
            step.getX() / period,
            step.getY() / period
        );
    }

    private Translation2d getTargetForce(Translation2d location) {
        var displacement = target.getTranslation().minus(location);
        double norm = displacement.getNorm();

        if (norm > 1e-6) {
            return new Translation2d(1.0 + 1.0 / (1e-6 + displacement.getNorm()), displacement.getAngle());
        } else {
            return Translation2d.kZero;
        }
    }

    private Translation2d getObstacleForce(Translation2d location) {
        double x = 0.0;
        double y = 0.0;

        for (Obstacle obstacle : obstacles) {
            var force = obstacle.forceAt(location, target.getTranslation());
            x += force.getX();
            y += force.getY();
        }

        return new Translation2d(x, y);
    }

    /**
     * A generic obstacle.
     */
    public abstract static class Obstacle {

        private final double strength;
        private final boolean positive;

        /**
         * Creates an obstacle.
         * @param strength The strength of the obstacle's force.
         * @param positive If the force is positive (pushes away) or negative (pulls towards).
         */
        public Obstacle(double strength, boolean positive) {
            this.strength = strength;
            this.positive = positive;
        }

        /**
         * Returns the force applied by the obstacle to the
         * robot, based on its current position and target.
         * @param position The robot's current position on the field.
         * @param target The current target field location.
         */
        public abstract Translation2d forceAt(Translation2d position, Translation2d target);

        /**
         * Converts a distance from the obstacle to the strength of the force.
         * @param dist The distance from the obstacle.
         */
        protected final double getForceMagnitude(double dist) {
            dist = Math.max(1e-3, Math.abs(dist));
            return (strength / (dist * dist)) * (positive ? 1.0 : -1.0);
        }
    }

    /**
     * A simple point obstacle. Applies a force from the specified field location.
     */
    public static final class PointObstacle extends Obstacle {

        private final Translation2d location;

        /**
         * Creates a point obstacle.
         * @param location The point's location on the field.
         * @param strength The strength of the obstacle's force.
         * @param positive If the force is positive (pushes away) or negative (pulls towards).
         */
        public PointObstacle(Translation2d location, double strength, boolean positive) {
            super(strength, positive);
            this.location = location;
        }

        @Override
        public Translation2d forceAt(Translation2d position, Translation2d target) {
            return new Translation2d(
                getForceMagnitude(location.getDistance(position)),
                position.minus(location).getAngle()
            );
        }
    }

    /**
     * An infinite line parallel to the Y axis that pushes parallel to the X axis.
     */
    public static final class VerticalObstacle extends Obstacle {

        private final double x;

        /**
         * Creates a vertical obstacle.
         * @param x The X coordinate of the line.
         * @param strength The strength of the obstacle's force.
         * @param positive If the force is positive (pushes away) or negative (pulls towards).
         */
        public VerticalObstacle(double x, double strength, boolean positive) {
            super(strength, positive);
            this.x = x;
        }

        @Override
        public Translation2d forceAt(Translation2d position, Translation2d target) {
            return new Translation2d(getForceMagnitude(x - position.getX()), 0.0);
        }
    }

    /**
     * An infinite line parallel to the X axis that pushes parallel to the Y axis.
     */
    public static final class HorizontalObstacle extends Obstacle {

        private final double y;

        /**
         * Creates a horizontal obstacle.
         * @param y The Y coordinate of the line.
         * @param strength The strength of the obstacle's force.
         * @param positive If the force is positive (pushes away) or negative (pulls towards).
         */
        public HorizontalObstacle(double y, double strength, boolean positive) {
            super(strength, positive);
            this.y = y;
        }

        @Override
        public Translation2d forceAt(Translation2d position, Translation2d target) {
            return new Translation2d(0.0, getForceMagnitude(y - position.getY()));
        }
    }

    public static final class LineObstacle extends Obstacle {

        private final Translation2d start;
        private final Translation2d end;

        private final double length;
        private final Rotation2d inverse;
        private final Rotation2d perpendicular;

        public LineObstacle(Translation2d start, Translation2d end, double strength, boolean positive) {
            super(strength, positive);
            this.start = start;
            this.end = end;

            var difference = end.minus(start);
            length = difference.getNorm();
            inverse = difference.getAngle().unaryMinus();
            perpendicular = difference.getAngle().rotateBy(Rotation2d.kCCW_Pi_2);
        }

        @Override
        public Translation2d forceAt(Translation2d position, Translation2d target) {
            var relative = position.minus(start).rotateBy(inverse);

            if (relative.getX() > 0.0 && relative.getX() < length) {
                return new Translation2d(
                    Math.copySign(getForceMagnitude(relative.getY()), relative.getY()),
                    perpendicular
                );
            }

            var closest = relative.getX() <= 0.0 ? start : end;
            return new Translation2d(
                getForceMagnitude(position.getDistance(closest)),
                position.minus(closest).getAngle()
            );
        }
    }

    public static final class TeardropObstacle extends Obstacle {

        final Translation2d location;
        final double primaryMaxRange;
        final double primaryRadius;
        final double tailStrength;
        final double tailLength;

        public TeardropObstacle(
            Translation2d location,
            double primaryStrength,
            double primaryMaxRange,
            double primaryRadius,
            double tailStrength,
            double tailLength,
            boolean positive
        ) {
            super(primaryStrength, positive);
            this.location = location;
            this.primaryMaxRange = primaryMaxRange;
            this.primaryRadius = primaryRadius;
            this.tailStrength = tailStrength;
            this.tailLength = tailLength + primaryMaxRange;
        }

        @Override
        public Translation2d forceAt(Translation2d position, Translation2d target) {
            var targetDiff = location.minus(target);
            var targetAngle = targetDiff.getAngle();
            var sidewaysPoint = new Translation2d(tailLength, targetDiff.getAngle()).plus(location);

            var positionToLocation = position.minus(location);
            var positionToLocationDistance = positionToLocation.getNorm();
            var outwardsForce = positionToLocationDistance <= primaryMaxRange
                ? new Translation2d(
                    getForceMagnitude(Math.max(positionToLocationDistance - primaryRadius, 0.0)),
                    positionToLocation.getAngle()
                )
                : Translation2d.kZero;

            var positionToLine = position.minus(location).rotateBy(targetAngle.unaryMinus());
            var distanceAlongLine = positionToLine.getX();

            Translation2d sidewaysForce = Translation2d.kZero;

            var distanceScalar = distanceAlongLine / tailLength;
            if (distanceScalar >= 0.0 && distanceScalar <= 1.0) {
                var secondaryMaxRange = MathUtil.interpolate(primaryMaxRange, 0.0, distanceScalar * distanceScalar);
                var distanceToLine = Math.abs(positionToLine.getY());

                if (distanceToLine <= secondaryMaxRange) {
                    double strength =
                        (distanceAlongLine < primaryMaxRange
                                ? tailStrength * (distanceAlongLine / primaryMaxRange)
                                : (-tailStrength * distanceAlongLine) / (tailLength - primaryMaxRange) +
                                (tailLength * tailStrength) / (tailLength - primaryMaxRange)) *
                        (1.0 - distanceToLine / secondaryMaxRange);

                    sidewaysForce = new Translation2d(
                        tailStrength *
                        strength *
                        (secondaryMaxRange - distanceToLine) *
                        Math.signum(
                            Math.sin(
                                target
                                    .minus(position)
                                    .getAngle()
                                    .minus(position.minus(sidewaysPoint).getAngle())
                                    .getRadians()
                            )
                        ),
                        targetAngle.rotateBy(Rotation2d.kCCW_90deg)
                    );
                }
            }

            return outwardsForce.plus(sidewaysForce);
        }
    }
}
