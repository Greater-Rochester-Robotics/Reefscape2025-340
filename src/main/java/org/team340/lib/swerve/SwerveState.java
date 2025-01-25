package org.team340.lib.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Represents the state of the robot's drivetrain.
 */
public final class SwerveState {

    /**
     * Contains information about swerve module states and positions.
     */
    public static final class Modules {

        /** The current measured module positions. */
        public final SwerveModulePosition[] positions;
        /** The current measured module states. */
        public final SwerveModuleState[] states;
        /** The last target states of the modules. */
        public final SwerveModuleState[] lastTarget;
        /** The next target states of the modules. */
        public final SwerveModuleState[] nextTarget;

        private Modules(SwerveModule[] modules) {
            positions = new SwerveModulePosition[modules.length];
            states = new SwerveModuleState[modules.length];
            lastTarget = new SwerveModuleState[modules.length];
            nextTarget = new SwerveModuleState[modules.length];
            for (int i = 0; i < modules.length; i++) {
                positions[i] = new SwerveModulePosition();
                states[i] = new SwerveModuleState();
                lastTarget[i] = modules[i].getLastTarget();
                nextTarget[i] = modules[i].getNextTarget();
            }
        }
    }

    /**
     * Represents the state of the odometry.
     */
    public static final class Odometry {

        /** If Phoenix timesync is being utilized. */
        public boolean timesync;
        /** The number of successful odometry measurements since the last loop. */
        public int successes;
        /** The number of failing odometry measurements since the last loop. */
        public int failures;

        private Odometry() {}
    }

    /** Information about module states and positions. */
    public final Modules modules;
    /** The state of the odometry. */
    public final Odometry odometry;
    /** The current measured robot-relative speeds. */
    public final ChassisSpeeds speeds;
    /** The next target robot-relative speeds. Updated when using {@code applySpeeds()}. */
    public final ChassisSpeeds targetSpeeds;
    /** The directionless measured velocity of the robot. */
    public double velocity;
    /** The robot's multi-turn yaw as reported by the IMU, in radians. */
    public double multiturnYaw;
    /** The robot's pitch as reported by the IMU. */
    public Rotation2d pitch;
    /** The robot's roll as reported by the IMU. */
    public Rotation2d roll;
    /** The current blue origin relative pose of the robot. */
    public Pose2d pose;

    SwerveState(SwerveModule[] modules) {
        this.modules = new Modules(modules);
        odometry = new Odometry();
        speeds = new ChassisSpeeds();
        targetSpeeds = new ChassisSpeeds();
        pitch = Rotation2d.kZero;
        roll = Rotation2d.kZero;
        pose = Pose2d.kZero;
    }
}
