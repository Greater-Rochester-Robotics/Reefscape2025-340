package org.team340.lib.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.hardware.SwerveIMUs.SwerveIMU;
import org.team340.lib.util.Math2;
import org.team340.lib.util.Sleep;
import org.team340.robot.Robot;

/**
 * An implementation of a swerve drivetrain, with support for various hardware.
 */
public class SwerveAPI implements AutoCloseable {

    public final SwerveState state;
    public final SwerveConfig config;

    final SwerveModule[] modules;
    final SwerveIMU imu;

    private final int moduleCount;
    private final double farthestModule;
    private final Translation2d[] moduleLocations;
    private final SwerveModuleState[] lockedStates;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private final PoseEstimator<SwerveModulePosition[]> poseEstimator;

    private final Lock odometryMutex = new ReentrantLock();
    private final SwerveOdometryThread odometryThread;

    private Rotation2d lastRobotAngle = Rotation2d.kZero;
    private double lastRatelimit = 0.0;

    private Consumer<ChassisSpeeds> imuSimHook = s -> {};

    /**
     * Create the drivetrain.
     * @param config The drivetrain's config.
     */
    public SwerveAPI(SwerveConfig config) {
        this.config = config;

        moduleCount = config.modules.length;
        modules = new SwerveModule[moduleCount];
        moduleLocations = new Translation2d[moduleCount];
        lockedStates = new SwerveModuleState[moduleCount];
        double farthest = 0.0;
        for (int i = 0; i < moduleCount; i++) {
            var moduleConfig = config.modules[i];
            modules[i] = new SwerveModule(config, moduleConfig);
            moduleLocations[i] = moduleConfig.location;
            lockedStates[i] = new SwerveModuleState(0.0, moduleLocations[i].getAngle());
            farthest = Math.max(farthest, moduleLocations[i].getNorm());
        }

        farthestModule = farthest;

        state = new SwerveState(modules);
        kinematics = new SwerveDriveKinematics(moduleLocations);
        odometry = new SwerveDriveOdometry(kinematics, Rotation2d.kZero, state.modules.positions);
        poseEstimator = new PoseEstimator<>(
            kinematics,
            odometry,
            config.odometryStdDevs,
            VecBuilder.fill(0.0, 0.0, 0.0)
        );

        imu = SwerveIMU.construct(config.imu, config, hook -> imuSimHook = hook);
        odometryThread = new SwerveOdometryThread();
    }

    /**
     * Refreshes inputs from all swerve hardware. This must be called periodically
     * in order for the API to function. Typically, this method is called at the
     * start of the swerve subsystem's {@code periodic()} method.
     */
    public void refresh() {
        odometryMutex.lock();
        try {
            odometryThread.run(true);
            state.timestamp = Timer.getFPGATimestamp();

            state.odometryThread.timesync = odometryThread.timesync;
            state.odometryThread.successes = odometryThread.successes;
            state.odometryThread.failures = odometryThread.failures;

            odometryThread.successes = 0;
            odometryThread.failures = 0;

            for (int i = 0; i < moduleCount; i++) {
                Math2.copyInto(modules[i].getPosition(), state.modules.positions[i]);
                Math2.copyInto(modules[i].getState(), state.modules.states[i]);
            }

            state.pose = poseEstimator.getEstimatedPosition();

            state.imu.yawMeasurements.clear();
            state.imu.yawMeasurements.addAll(odometryThread.yawMeasurements);
            odometryThread.yawMeasurements.clear();

            if (!state.imu.yawMeasurements.isEmpty()) {
                state.imu.yaw = state.imu.yawMeasurements.get(state.imu.yawMeasurements.size() - 1).yaw();
            }
        } finally {
            odometryMutex.unlock();
        }

        Math2.copyInto(kinematics.toChassisSpeeds(state.modules.states), state.speeds);
        state.velocity = Math.hypot(state.speeds.vxMetersPerSecond, state.speeds.vyMetersPerSecond);
        state.translation = state.pose.getTranslation();
        state.rotation = state.pose.getRotation();

        state.imu.pitch = imu.getPitch();
        state.imu.roll = imu.getRoll();

        imuSimHook.accept(state.speeds);
    }

    /**
     * Adds vision measurements to the pose estimator.
     * @param measurements Vision measurements to apply to the pose estimator.
     */
    public void addVisionMeasurements(VisionMeasurement... measurements) {
        odometryMutex.lock();
        try {
            for (VisionMeasurement measurement : measurements) {
                if (measurement.stdDevs == null) {
                    poseEstimator.addVisionMeasurement(measurement.visionPose, measurement.timestamp);
                } else {
                    poseEstimator.addVisionMeasurement(
                        measurement.visionPose,
                        measurement.timestamp,
                        measurement.stdDevs
                    );
                }
            }

            state.pose = poseEstimator.getEstimatedPosition();
        } finally {
            odometryMutex.unlock();
        }
    }

    /**
     * Resets the pose of the robot, inherently seeding field-relative movement.
     * Additionally, odometry and vision measurement history is flushed. This
     * method is typically invoked at the start of a match to set the robot's
     * position to the starting location of an autonomous mode. The supplied
     * pose is expected to be blue origin relative.
     * @param pose The new blue origin relative pose to apply to the pose estimator.
     */
    public void resetPose(Pose2d pose) {
        odometryMutex.lock();
        try {
            poseEstimator.resetPosition(odometryThread.lastYaw, state.modules.positions, pose);
            state.pose = poseEstimator.getEstimatedPosition();
        } finally {
            odometryMutex.unlock();
        }
    }

    /**
     * Tares the rotation of the robot. Useful for fixing an out of sync or drifting
     * IMU. For most cases, a perspective of {@link Perspective#kOperator} is
     * desirable. {@link Perspective#kRobot} will no-op.
     * @param perspective The perspective to tare the rotation to.
     */
    public void tareRotation(Perspective perspective) {
        var rotation = perspective.getTareRotation();
        if (rotation == null) return;
        odometryMutex.lock();
        try {
            poseEstimator.resetRotation(rotation);
            state.pose = poseEstimator.getEstimatedPosition();
        } finally {
            odometryMutex.unlock();
        }
    }

    /**
     * Drives using inputs from the driver's controller. The {@code x} and {@code y} parameters
     * expect the controller's NED (north-east-down) convention, and will automatically convert
     * to WPILib's typical NWU (north-west-up) convention when applying chassis speeds.
     * @param x The X value of the driver's joystick, from {@code [-1.0, 1.0]}.
     * @param y The Y value of the driver's joystick, from {@code [-1.0, 1.0]}.
     * @param angular The CCW+ angular speed to apply, from {@code [-1.0, 1.0]}.
     * @param perspective The forward perspective for the chassis speeds.
     * @param discretize If the generated speeds should be discretized.
     * @param ratelimit If the robot's acceleration should be constrained.
     */
    public void applyDriverInput(
        double x,
        double y,
        double angular,
        Perspective perspective,
        boolean discretize,
        boolean ratelimit
    ) {
        applyAssistedDriverInput(x, y, angular, new ChassisSpeeds(), perspective, discretize, ratelimit);
    }

    /**
     * Drives using inputs from the driver's controller, with a specified additional chassis velocity.
     * The {@code x} and {@code y} parameters expect the controller's NED (north-east-down) convention,
     * and will automatically convert to WPILib's typical NWU (north-west-up) convention when applying
     * chassis speeds.
     * @param x The X value of the driver's joystick, from {@code [-1.0, 1.0]}.
     * @param y The Y value of the driver's joystick, from {@code [-1.0, 1.0]}.
     * @param angular The CCW+ angular speed to apply, from {@code [-1.0, 1.0]}.
     * @param assist Additional velocities to apply.
     * @param perspective The forward perspective for the chassis speeds.
     * @param discretize If the generated speeds should be discretized.
     * @param ratelimit If the robot's acceleration should be constrained.
     */
    public void applyAssistedDriverInput(
        double x,
        double y,
        double angular,
        ChassisSpeeds assist,
        Perspective perspective,
        boolean discretize,
        boolean ratelimit
    ) {
        double k = 0.0;
        double norm = Math.hypot(x, y);
        double deadband = config.driverVelDeadband;

        if (norm >= deadband) {
            k = ((norm - deadband) / (1.0 - deadband)) / norm;
        }

        x *= k;
        y *= k;
        angular = MathUtil.applyDeadband(angular, config.driverAngularVelDeadband);

        double xyMult = config.driverVel * Math.pow(Math.hypot(x, y), config.driverVelExp - 1.0);
        double angularVel =
            config.driverAngularVel * Math.copySign(Math.pow(angular, config.driverAngularVelExp), angular);

        ChassisSpeeds speeds = new ChassisSpeeds(
            (-y * xyMult) + assist.vxMetersPerSecond,
            (-x * xyMult) + assist.vyMetersPerSecond,
            angularVel + assist.omegaRadiansPerSecond
        );

        applySpeeds(speeds, perspective, discretize, ratelimit);
    }

    /**
     * Drives using chassis speeds.
     * @param speeds The chassis speeds to apply. Note that the provided {@link ChassisSpeeds} object may be mutated.
     * @param perspective The forward perspective for the chassis speeds.
     * @param discretize If the speeds should be discretized.
     * @param ratelimit If the robot's acceleration should be constrained.
     */
    public void applySpeeds(ChassisSpeeds speeds, Perspective perspective, boolean discretize, boolean ratelimit) {
        double w_max = config.velocity / farthestModule;
        if (Math.abs(speeds.omegaRadiansPerSecond) >= w_max) {
            speeds.vxMetersPerSecond = 0.0;
            speeds.vyMetersPerSecond = 0.0;
            speeds.omegaRadiansPerSecond = Math.copySign(w_max, speeds.omegaRadiansPerSecond);
        } else {
            ChassisSpeeds relative = perspective.toRobotSpeeds(speeds, state.pose.getRotation());

            double vx = relative.vxMetersPerSecond;
            double vy = relative.vyMetersPerSecond;
            double w = relative.omegaRadiansPerSecond;

            double k = 1.0;
            double v_max2 = config.velocity * config.velocity;
            double fastest = v_max2;

            for (var r : moduleLocations) {
                double vx_w = -w * r.getY();
                double vy_w = w * r.getX();

                double vx_m = vx + vx_w;
                double vy_m = vy + vy_w;
                double v_m2 = vx_m * vx_m + vy_m * vy_m;

                if (v_m2 > fastest) {
                    double a = vx * vx + vy * vy;
                    double b = 2 * vx * vx_w + 2 * vy * vy_w;
                    double c = vx_w * vx_w + vy_w * vy_w - v_max2;
                    k = (2 * c) / (-b - Math.sqrt(b * b - 4 * a * c));
                    fastest = v_m2;
                }
            }

            speeds.vxMetersPerSecond *= k;
            speeds.vyMetersPerSecond *= k;
        }

        if (ratelimit) {
            double now = Timer.getFPGATimestamp();

            if (now - lastRatelimit < config.period * 2.0) {
                ChassisSpeeds lastSpeeds = perspective.toPerspectiveSpeeds(state.targetSpeeds, lastRobotAngle);

                double vx_l = lastSpeeds.vxMetersPerSecond;
                double vy_l = lastSpeeds.vyMetersPerSecond;
                double v_l = Math.hypot(vx_l, vy_l);
                double w_l = lastSpeeds.omegaRadiansPerSecond;

                double dx = speeds.vxMetersPerSecond - vx_l;
                double dy = speeds.vyMetersPerSecond - vy_l;
                double a_slip = config.slipAccel * config.period;
                if (dx * dx + dy * dy > a_slip * a_slip) {
                    double k = a_slip / Math.hypot(dx, dy);
                    speeds.vxMetersPerSecond = vx_l + (k * dx);
                    speeds.vyMetersPerSecond = vy_l + (k * dy);
                }

                double v = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
                double a_torque = (config.torqueAccel * config.period) * (1.0 - (v_l / config.velocity));
                if (v - v_l > a_torque) {
                    double k = (v_l + a_torque) / v;
                    speeds.vxMetersPerSecond *= k;
                    speeds.vyMetersPerSecond *= k;
                }

                double dw = speeds.omegaRadiansPerSecond - w_l;
                double a_angular = config.angularAccel * config.period;
                speeds.omegaRadiansPerSecond = w_l + (Math.min(a_angular / Math.abs(dw), 1.0) * dw);
            }

            lastRatelimit = now;
        }

        if (discretize) {
            double dtheta = speeds.omegaRadiansPerSecond * config.discretizationPeriod;

            double sin = -dtheta / 2.0;
            double cos = Math2.epsilonEquals(Math.cos(dtheta) - 1.0, 0.0)
                ? 1.0 - ((1.0 / 12.0) * dtheta * dtheta)
                : (sin * Math.sin(dtheta)) / (Math.cos(dtheta) - 1.0);

            double dt = config.period;
            double dx = speeds.vxMetersPerSecond * dt;
            double dy = speeds.vyMetersPerSecond * dt;

            speeds.vxMetersPerSecond = ((dx * cos) - (dy * sin)) / dt;
            speeds.vyMetersPerSecond = ((dx * sin) + (dy * cos)) / dt;
        }

        lastRobotAngle = state.pose.getRotation();
        Math2.copyInto(perspective.toRobotSpeeds(speeds, state.pose.getRotation()), state.targetSpeeds);
        applyStates(kinematics.toSwerveModuleStates(state.targetSpeeds));
    }

    /**
     * Drives using module states.
     * @param states The states to apply.
     */
    public void applyStates(SwerveModuleState[] states) {
        if (moduleCount != states.length) {
            throw new IllegalArgumentException(
                "Requested " + states.length + " states be applied to " + moduleCount + " modules"
            );
        }

        for (int i = 0; i < moduleCount; i++) {
            modules[i].applyState(states[i]);
        }
    }

    /**
     * Drives the modules to stop the robot from moving.
     * @param lock If the wheels should be driven to an X formation to stop the robot from being pushed.
     */
    public void applyStop(boolean lock) {
        lastRatelimit = Timer.getFPGATimestamp();
        Math2.zero(state.targetSpeeds);

        for (int i = 0; i < moduleCount; i++) {
            SwerveModuleState state;
            if (lock) {
                state = lockedStates[i];
            } else {
                state = modules[i].getNextTarget();
                state.speedMetersPerSecond = 0.0;
            }

            modules[i].applyState(state);
        }
    }

    /**
     * Drives the robot using open-loop voltage. Intended for characterization.
     * Plumbing for recording device voltage via their Java API is intentionally
     * unavailable, as GC pressure and CAN latency will result in inaccurate data.
     * Use Phoenix Signal Logging or URCL instead.
     * @param voltage The voltage to apply to the move motors.
     * @param angle The robot-relative angle to apply to the turn motors.
     */
    public void applyVoltage(double voltage, Rotation2d angle) {
        for (int i = 0; i < moduleCount; i++) {
            modules[i].applyVoltage(voltage, angle);
        }
    }

    /**
     * Enables publishing tunables for adjustment of the API's constants.
     * @param name The parent name for the tunables in NetworkTables.
     */
    public void enableTunables(String name) {
        SwerveTunables.initialize(name, this);
    }

    @Override
    public void close() {
        try {
            odometryThread.close();
            for (var module : modules) module.close();
            imu.close();
        } catch (Exception e) {}
    }

    /**
     * Represents a measurement from vision to apply to the pose estimator.
     * @see {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}.
     */
    @Logged(strategy = Strategy.OPT_IN)
    public static final record VisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
        /**
         * Represents a measurement from vision to apply to the pose estimator.
         * @see {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}.
         */
        public VisionMeasurement(Pose2d visionPose, double timestamp) {
            this(visionPose, timestamp, null);
        }
    }

    /**
     * Contains a yaw measurement alongside the timestamp of the measurement, in seconds.
     */
    public static final record TimestampedYaw(Rotation2d yaw, double timestamp) {}

    /**
     * Manages swerve odometry. Will run asynchronously at the configured odometry update
     * period, unless the configured period is the same or more than the main robot loop
     * period. The {@link SwerveOdometryThread#run(boolean)} method is also invoked in
     * {@link SwerveAPI#refresh()}, to ensure the latest measurements are applied to
     * pose estimation when executing user logic in the main loop.
     */
    private final class SwerveOdometryThread implements AutoCloseable {

        public final List<TimestampedYaw> yawMeasurements = new ArrayList<>();
        public Rotation2d lastYaw = Rotation2d.kZero;
        public boolean timesync = false;
        public int successes = 0;
        public int failures = 0;

        private final SwerveModulePosition[] positionCache;
        private final BaseStatusSignal[] signals;
        private final Thread thread;

        private volatile boolean active = false;
        private double lastTime = 0.0;

        public SwerveOdometryThread() {
            List<BaseStatusSignal> signalList = new ArrayList<>();
            signalList.addAll(imu.getSignals());
            for (var module : modules) signalList.addAll(module.getSignals());
            signals = signalList.stream().toArray(BaseStatusSignal[]::new);
            positionCache = new SwerveModulePosition[moduleCount];
            for (int i = 0; i < moduleCount; i++) {
                positionCache[i] = modules[i].getPosition();
            }

            if (config.odometryPeriod < config.period) {
                thread = new Thread(() -> {
                    Threads.setCurrentThreadPriority(true, 1);
                    while (active) this.run(false);
                });
                thread.setName("SwerveAPI");
                thread.setDaemon(true);
                timesync = config.phoenixPro && config.phoenixCanBus.isNetworkFD();
                active = true;
                thread.start();
            } else {
                thread = null;
            }
        }

        /**
         * Runs an odometry update.
         * @param sync If the update is being invoked from the main loop.
         */
        public void run(boolean sync) {
            StatusCode phoenixStatus = StatusCode.OK;
            if (!sync) {
                if (timesync) {
                    phoenixStatus = BaseStatusSignal.waitForAll(config.period, signals);
                } else {
                    Sleep.seconds(Math.max(0.0, config.odometryPeriod - (Timer.getFPGATimestamp() - lastTime)));
                    lastTime = Timer.getFPGATimestamp();
                }
            }

            odometryMutex.lock();
            try {
                if (!timesync && signals.length > 0) phoenixStatus = BaseStatusSignal.refreshAll(signals);

                lastYaw = imu.getYaw();
                double yawTimestamp = Timer.getFPGATimestamp();

                boolean readError = !phoenixStatus.isOK() || imu.readError();
                for (var module : modules) {
                    if (!module.refresh()) readError = true;
                }

                if (readError) {
                    failures++;
                    if (!Robot.isSimulation()) return;
                }

                poseEstimator.update(lastYaw, positionCache);
                yawMeasurements.add(new TimestampedYaw(odometry.getPoseMeters().getRotation(), yawTimestamp));
                successes++;
            } finally {
                odometryMutex.unlock();
            }
        }

        @Override
        public void close() {
            if (active) {
                active = false;
                if (thread != null && thread.isAlive()) {
                    try {
                        thread.interrupt();
                        thread.join();
                    } catch (Exception e) {
                        Thread.currentThread().interrupt();
                    }
                }
            }
        }
    }
}
