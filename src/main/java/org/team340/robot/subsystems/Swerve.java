package org.team340.robot.subsystems;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Orchestra;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.team340.lib.swerve.Perspective;
import org.team340.lib.swerve.SwerveAPI;
import org.team340.lib.swerve.SwerveAPI.VisionMeasurement;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.SwerveEncoders;
import org.team340.lib.swerve.hardware.SwerveIMUs;
import org.team340.lib.swerve.hardware.SwerveMotors;
import org.team340.lib.util.Alliance;
import org.team340.lib.util.Math2;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.robot.Constants;
import org.team340.robot.Constants.Cameras;
import org.team340.robot.Constants.FieldConstants;
import org.team340.robot.Constants.LowerCAN;
import org.team340.robot.util.VisionEstimator;
import org.team340.robot.util.VisionEstimator.VisionEstimates;

/**
 * The robot's swerve drivetrain.
 */
@Logged
public final class Swerve extends GRRSubsystem {

    private static final double kMoveRatio = (54.0 / 10.0) * (18.0 / 38.0) * (45.0 / 15.0);
    private static final double kTurnRatio = (22.0 / 10.0) * (88.0 / 16.0);

    private static final Orchestra kOrchestra = new Orchestra();

    private static final SwerveModuleConfig kFrontLeft = new SwerveModuleConfig()
        .setName("frontLeft")
        .setLocation(0.28, 0.28)
        .setMoveMotor(SwerveMotors.talonFX(LowerCAN.kFlMove, true))
        .setTurnMotor(SwerveMotors.talonFX(LowerCAN.kFlTurn, true))
        .setEncoder(SwerveEncoders.canCoder(LowerCAN.kFlEncoder, 0.296, false));

    private static final SwerveModuleConfig kFrontRight = new SwerveModuleConfig()
        .setName("frontRight")
        .setLocation(0.28, -0.28)
        .setMoveMotor(SwerveMotors.talonFX(LowerCAN.kFrMove, true))
        .setTurnMotor(SwerveMotors.talonFX(LowerCAN.kFrTurn, true))
        .setEncoder(SwerveEncoders.canCoder(LowerCAN.kFrEncoder, -0.395, false));

    private static final SwerveModuleConfig kBackLeft = new SwerveModuleConfig()
        .setName("backLeft")
        .setLocation(-0.28, 0.28)
        .setMoveMotor(SwerveMotors.talonFX(LowerCAN.kBlMove, true))
        .setTurnMotor(SwerveMotors.talonFX(LowerCAN.kBlTurn, true))
        .setEncoder(SwerveEncoders.canCoder(LowerCAN.kBlEncoder, 0.190, false));

    private static final SwerveModuleConfig kBackRight = new SwerveModuleConfig()
        .setName("backRight")
        .setLocation(-0.28, -0.28)
        .setMoveMotor(SwerveMotors.talonFX(LowerCAN.kBrMove, true))
        .setTurnMotor(SwerveMotors.talonFX(LowerCAN.kBrTurn, true))
        .setEncoder(SwerveEncoders.canCoder(LowerCAN.kBrEncoder, -0.079, false));

    private static final SwerveConfig kConfig = new SwerveConfig()
        .setTimings(TimedRobot.kDefaultPeriod, 0.004, 0.02)
        .setMovePID(0.27, 0.0, 0.0)
        .setMoveFF(0.0, 0.126)
        .setTurnPID(100.0, 0.0, 0.2)
        .setBrakeMode(false, true)
        .setLimits(4.0, 17.5, 14.0, 30.0)
        .setDriverProfile(4.0, 1.5, 0.15, 4.2, 2.0, 0.05)
        .setPowerProperties(Constants.kVoltage, 100.0, 80.0, 60.0, 60.0)
        .setMechanicalProperties(kMoveRatio, kTurnRatio, 0.0, Units.inchesToMeters(4.0))
        .setOdometryStd(0.1, 0.1, 0.1)
        .setIMU(SwerveIMUs.canandgyro(LowerCAN.kCanandgyro))
        .setPhoenixFeatures(new CANBus(LowerCAN.kLowerCANBus), true, true, true, kOrchestra)
        .setModules(kFrontLeft, kFrontRight, kBackLeft, kBackRight);

    private static final TunableDouble kFaceReefTolerance = Tunable.doubleValue("swerve/kFaceReefTolerance", 1.5);

    private final SwerveAPI api;

    private final PIDController autoPIDx;
    private final PIDController autoPIDy;
    private final PIDController autoPIDangular;

    private final ProfiledPIDController faceReefPID;

    private final VisionEstimator[] visionEstimators;
    private final List<Pose2d> estimates = new ArrayList<>();
    private final List<Pose3d> targets = new ArrayList<>();

    private Pose2d autoLast = null;
    private Pose2d autoNext = null;
    private boolean facingReef = true;

    public Swerve() {
        api = new SwerveAPI(kConfig);

        autoPIDx = new PIDController(7.0, 0.0, 0.0);
        autoPIDy = new PIDController(7.0, 0.0, 0.0);
        autoPIDangular = new PIDController(5.0, 0.0, 0.0);
        autoPIDangular.enableContinuousInput(-Math.PI, Math.PI);

        faceReefPID = new ProfiledPIDController(10.0, 0.5, 0.25, new Constraints(10.0, 30.0));
        faceReefPID.enableContinuousInput(-Math.PI, Math.PI);
        faceReefPID.setIZone(0.8);

        api.enableTunables("swerve/api");
        Tunable.pidController("swerve/autoPID", autoPIDx);
        Tunable.pidController("swerve/autoPID", autoPIDy);
        Tunable.pidController("swerve/autoPIDangular", autoPIDangular);
        Tunable.pidController("swerve/faceReefPID", faceReefPID);

        visionEstimators = new VisionEstimator[] {
            // new VisionEstimator("middle", Cameras.kMiddle),
            new VisionEstimator("left", Cameras.kLeft),
            new VisionEstimator("right", Cameras.kRight)
            // new VisionEstimator("back", Cameras.kBack)
        };
    }

    @Override
    public void periodic() {
        api.refresh();

        // TODO before merge, disable all vision code using tunable boolean
        // resetting local measurements
        estimates.clear();
        targets.clear();

        List<VisionMeasurement> measurements = new ArrayList<>();
        for (VisionEstimator estimator : visionEstimators) {
            VisionEstimates result = estimator.getUnreadResults(api.state.pose, api.state.timestamp);

            measurements.addAll(result.measurements());
            estimates.addAll(result.getPoses());
            targets.addAll(result.targets());
        }

        api.addVisionMeasurements(measurements.stream().toArray(VisionMeasurement[]::new));
    }

    /**
     * Returns the current blue origin relative pose of the robot.
     */
    @NotLogged
    public Pose2d getPose() {
        return api.state.pose;
    }

    /**
     * Returns true if the elevator and goose neck are safe
     * to move, based on the robot's position on the field.
     */
    public boolean safeForGoose() {
        // TODO
        return facingReef;
    }

    /**
     * Tares the rotation of the robot. Useful for
     * fixing an out of sync or drifting IMU.
     */
    public Command tareRotation() {
        return commandBuilder("Swerve.tareRotation()")
            .onInitialize(() -> api.tareRotation(Perspective.kOperator))
            .isFinished(true)
            .ignoringDisable(true);
    }

    /**
     * Drives the robot using driver input.
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     * @param angular The CCW+ angular speed to apply, from {@code [-1.0, 1.0]}.
     */
    public Command drive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier angular) {
        return commandBuilder("Swerve.drive()").onExecute(() ->
            api.applyDriverInput(
                x.getAsDouble(),
                y.getAsDouble(),
                angular.getAsDouble(),
                Perspective.kOperator,
                true,
                true
            )
        );
    }

    /**
     * Drives the robot using driver input while facing the reef.
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     */
    public Command driveReef(DoubleSupplier x, DoubleSupplier y) {
        return commandBuilder("Swerve.driveReef()")
            .onInitialize(() -> faceReefPID.reset(api.state.yaw.getRadians(), api.state.speeds.omegaRadiansPerSecond))
            .onExecute(() -> {
                Translation2d reefCenter = Alliance.isBlue()
                    ? FieldConstants.kReefCenterBlue
                    : FieldConstants.kReefCenterRed;

                Rotation2d angle = reefCenter.minus(api.state.translation).getAngle();
                double target =
                    Math.floor(angle.plus(new Rotation2d(Math2.kSixthPi)).getRadians() / Math2.kThirdPi) *
                    Math2.kThirdPi;

                facingReef = Math2.epsilonEquals(target, api.state.yaw.getRadians(), kFaceReefTolerance.value());

                double angularVel = faceReefPID.calculate(api.state.yaw.getRadians(), target);
                api.applyDriverXY(x.getAsDouble(), y.getAsDouble(), angularVel, Perspective.kOperator, true, true);
            })
            .onEnd(() -> facingReef = true);
    }

    /**
     * Drives the modules to stop the robot from moving.
     * @param lock If the wheels should be driven to an X formation to stop the robot from being pushed.
     */
    public Command stop(boolean lock) {
        return commandBuilder("Swerve.stop(" + lock + ")").onExecute(() -> api.applyStop(lock));
    }

    /**
     * Stops the robot from moving, and cleans up auto-related telemetry.
     * This command should be ran at the end of an autonomous routine.
     */
    public Command finishAuto() {
        return commandBuilder("Swerve.finishAuto()")
            .onInitialize(() -> {
                autoLast = null;
                autoNext = autoLast;
            })
            .onExecute(() -> api.applyStop(false));
    }

    /**
     * Resets the pose of the robot, inherently seeding field-relative movement. This
     * method is not intended for use outside of creating an {@link AutoFactory}.
     * @param pose The new blue origin relative pose to apply to the pose estimator.
     */
    public void resetPose(Pose2d pose) {
        api.resetPose(pose);
    }

    /**
     * Follows a Choreo trajectory by moving towards the next sample. This method
     * is not intended for use outside of creating an {@link AutoFactory}.
     * @param sample The next trajectory sample.
     */
    public void followTrajectory(SwerveSample sample) {
        autoLast = autoNext;
        autoNext = sample.getPose();

        Pose2d pose = api.state.pose;
        api.applySpeeds(
            new ChassisSpeeds(
                sample.vx + autoPIDx.calculate(pose.getX(), sample.x),
                sample.vy + autoPIDy.calculate(pose.getY(), sample.y),
                sample.omega + autoPIDangular.calculate(pose.getRotation().getRadians(), sample.heading)
            ),
            Perspective.kBlue,
            true,
            false
        );
    }
}
