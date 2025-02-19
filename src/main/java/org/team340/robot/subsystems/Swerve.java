package org.team340.robot.subsystems;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
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
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.robot.Constants;
import org.team340.robot.Constants.FieldConstants;
import org.team340.robot.Constants.LowerCAN;

/**
 * The robot's swerve drivetrain.
 */
@Logged
public final class Swerve extends GRRSubsystem {

    private static final double kMoveRatio = (54.0 / 10.0) * (18.0 / 38.0) * (45.0 / 15.0);
    private static final double kTurnRatio = (22.0 / 10.0) * (88.0 / 16.0);

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
        .setPhoenixFeatures(new CANBus(LowerCAN.kLowerCANBus), true, true, true)
        .setModules(kFrontLeft, kFrontRight, kBackLeft, kBackRight);

    private final SwerveAPI api;

    private final PIDController autoPIDx;
    private final PIDController autoPIDy;
    private final PIDController autoPIDangular;

    private final ProfiledPIDController faceReefPID;

    private final AprilTagFieldLayout aprilTags;
    private final PhotonCamera[] cameras;
    private final Transform3d[] cameraLocations;
    private final List<Pose2d> measurements = new ArrayList<>();
    private final List<Pose3d> targets = new ArrayList<>();

    private Pose2d autoLast = null;
    private Pose2d autoNext = null;

    // TODO fill in
    private static final Transform3d kBackLeftCamera = new Transform3d(
        new Translation3d(-0.29153, 0.19629, 0.24511),
        new Rotation3d(0.0, Math.toRadians(-22.0), Math.toRadians(155.0))
    );
    private static final Transform3d kBackRightCamera = new Transform3d(
        new Translation3d(-0.29153, -0.19629, 0.24511),
        new Rotation3d(0.0, Math.toRadians(-22.0), Math.toRadians(-155.0))
    );
    private static final Transform3d kFrontLeftCamera = new Transform3d(
        new Translation3d(-0.29153, -0.19629, 0.24511),
        new Rotation3d(0.0, Math.toRadians(-22.0), Math.toRadians(-155.0))
    );
    private static final Transform3d kFrontRightCamera = new Transform3d(
        new Translation3d(-0.29153, -0.19629, 0.24511),
        new Rotation3d(0.0, Math.toRadians(-22.0), Math.toRadians(-155.0))
    );

    public Swerve() {
        api = new SwerveAPI(kConfig);

        autoPIDx = new PIDController(7.0, 0.0, 0.0);
        autoPIDy = new PIDController(7.0, 0.0, 0.0);
        autoPIDangular = new PIDController(5.0, 0.0, 0.0);
        autoPIDangular.enableContinuousInput(-Math.PI, Math.PI);

        faceReefPID = new ProfiledPIDController(6.25, 0.5, 0.0, new Constraints(9.5, 24.0));
        faceReefPID.enableContinuousInput(-Math.PI, Math.PI);
        faceReefPID.setIZone(0.8);

        api.enableTunables("swerve/api");
        Tunable.pidController("swerve/autoPID", autoPIDx);
        Tunable.pidController("swerve/autoPID", autoPIDy);
        Tunable.pidController("swerve/autoPIDangular", autoPIDangular);

        // TODO update to 2025 field
        aprilTags = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        aprilTags.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        cameras = new PhotonCamera[] {
            new PhotonCamera("backLeft"),
            new PhotonCamera("backRight"),
            new PhotonCamera("frontLeft"),
            new PhotonCamera("frontRight")
        };
        cameraLocations = new Transform3d[] { kBackLeftCamera, kBackRightCamera, kFrontLeftCamera, kFrontRightCamera };
    }

    @Override
    public void periodic() {
        api.refresh();

        // TODO before merge, disable all vision code using tunable boolean
        // resetting local measurements
        measurements.clear();
        targets.clear();

        // update local pose and targets based on all cameras and april tags
        for (int i = 0; i < cameras.length; i++) {
            // get all results at once because they will change over time
            // TODO use different update function to grab all results since last update, loop over all results
            // TODO split out into different methods
            var result = cameras[i].getLatestResult();
            for (var target : result.getTargets()) {
                // TODO check for null, save to vector, use other addVisionMeasurement to update all at once
                calculateVisionMeasurement(target, cameraLocations[i], result.getTimestampSeconds());
            }
        }
    }

    /**
     * Returns whether april tags should be weighted higher
     * @param id april tag id
     * @return true if tag should be weighted higher
     */
    @NotLogged
    private boolean determineIsImportant(int id) {
        // TODO replace with configurable id numbers
        return id == 3 || id == 4 || id == 7 || id == 8;
    }

    private VisionMeasurement calculateVisionMeasurement(
        PhotonTrackedTarget target,
        Transform3d cameraLocation,
        double timeStamp
    ) {
        // skip calculation for bad targets
        if (target.getPoseAmbiguity() > 0.09) return null;

        // checks what april tag we are looking at
        int id = target.getFiducialId();
        boolean important = determineIsImportant(id);
        Optional<Pose3d> tagPose = aprilTags.getTagPose(id);
        if (tagPose.isEmpty()) return null;

        double distance = api.state.pose.getTranslation().getDistance(tagPose.get().getTranslation().toTranslation2d());

        Pose3d estimate = PhotonUtils.estimateFieldToRobotAprilTag(
            target.getBestCameraToTarget(),
            tagPose.get(),
            cameraLocation
        );

        // if flying abort
        if (estimate.getZ() > 0.75) return null;

        // update all measurements, weight important april tags higher
        double stdScale = Math.pow(distance * (important ? 0.5 : 1.0), 2.0);
        double xyStd = 0.2 * stdScale; // TODO no magic numbers
        double angStd = 0.3 * stdScale;

        api.addVisionMeasurement(estimate.toPose2d(), timeStamp, VecBuilder.fill(xyStd, xyStd, angStd));

        targets.add(tagPose.get());
        measurements.add(estimate.toPose2d());
        return new VisionMeasurement(estimate.toPose2d(), timeStamp);
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
        return true;
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

                double angularVel = faceReefPID.calculate(api.state.yaw.getRadians(), target);
                api.applyDriverInput(x.getAsDouble(), y.getAsDouble(), angularVel, Perspective.kOperator, true, true);
            });
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
