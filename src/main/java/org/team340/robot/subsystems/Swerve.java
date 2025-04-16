package org.team340.robot.subsystems;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.team340.lib.swerve.Perspective;
import org.team340.lib.swerve.SwerveAPI;
import org.team340.lib.swerve.SwerveState;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.SwerveEncoders;
import org.team340.lib.swerve.hardware.SwerveIMUs;
import org.team340.lib.swerve.hardware.SwerveMotors;
import org.team340.lib.util.Alliance;
import org.team340.lib.util.Math2;
import org.team340.lib.util.Mutable;
import org.team340.lib.util.Profiler;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.robot.Constants;
import org.team340.robot.Constants.FieldConstants;
import org.team340.robot.Constants.FieldConstants.ReefLocation;
import org.team340.robot.Constants.LowerCAN;
import org.team340.robot.Constants.UpperCAN;
import org.team340.robot.util.RepulsorField;
import org.team340.robot.util.RepulsorField.RepulsorSample;
import org.team340.robot.util.VisionManager;

/**
 * The robot's swerve drivetrain.
 */
@Logged
public final class Swerve extends GRRSubsystem {

    private static final double kMoveRatio = (54.0 / 12.0) * (18.0 / 38.0) * (45.0 / 15.0);
    private static final double kTurnRatio = (22.0 / 10.0) * (88.0 / 16.0);
    private static final double kModuleOffset = Units.inchesToMeters(12.5);

    private static final SwerveModuleConfig kFrontLeft = new SwerveModuleConfig()
        .setName("frontLeft")
        .setLocation(kModuleOffset, kModuleOffset)
        .setMoveMotor(SwerveMotors.talonFX(LowerCAN.kFlMove, true))
        .setTurnMotor(SwerveMotors.talonFX(LowerCAN.kFlTurn, true))
        .setEncoder(SwerveEncoders.canCoder(LowerCAN.kFlEncoder, 0.296, false));

    private static final SwerveModuleConfig kFrontRight = new SwerveModuleConfig()
        .setName("frontRight")
        .setLocation(kModuleOffset, -kModuleOffset)
        .setMoveMotor(SwerveMotors.talonFX(LowerCAN.kFrMove, true))
        .setTurnMotor(SwerveMotors.talonFX(LowerCAN.kFrTurn, true))
        .setEncoder(SwerveEncoders.canCoder(LowerCAN.kFrEncoder, -0.395, false));

    private static final SwerveModuleConfig kBackLeft = new SwerveModuleConfig()
        .setName("backLeft")
        .setLocation(-kModuleOffset, kModuleOffset)
        .setMoveMotor(SwerveMotors.talonFX(LowerCAN.kBlMove, true))
        .setTurnMotor(SwerveMotors.talonFX(LowerCAN.kBlTurn, true))
        .setEncoder(SwerveEncoders.canCoder(LowerCAN.kBlEncoder, 0.190, false));

    private static final SwerveModuleConfig kBackRight = new SwerveModuleConfig()
        .setName("backRight")
        .setLocation(-kModuleOffset, -kModuleOffset)
        .setMoveMotor(SwerveMotors.talonFX(LowerCAN.kBrMove, true))
        .setTurnMotor(SwerveMotors.talonFX(LowerCAN.kBrTurn, true))
        .setEncoder(SwerveEncoders.canCoder(LowerCAN.kBrEncoder, -0.079, false));

    private static final SwerveConfig kConfig = new SwerveConfig()
        .setTimings(TimedRobot.kDefaultPeriod, 0.004, 0.02, 0.01)
        .setMovePID(0.3, 0.0, 0.0)
        .setMoveFF(0.15, 0.128)
        .setTurnPID(100.0, 0.0, 0.2)
        .setBrakeMode(false, true)
        .setLimits(4.5, 0.05, 16.5, 11.5, 28.0)
        .setDriverProfile(4.5, 1.5, 0.15, 4.7, 2.0, 0.05)
        .setPowerProperties(Constants.kVoltage, 100.0, 80.0, 60.0, 60.0)
        .setMechanicalProperties(kMoveRatio, kTurnRatio, 0.0, Units.inchesToMeters(4.0))
        .setOdometryStd(0.1, 0.1, 0.05)
        .setIMU(SwerveIMUs.canandgyro(UpperCAN.kCanandgyro))
        .setPhoenixFeatures(new CANBus(LowerCAN.kLowerCANBus), true, true, true)
        .setModules(kFrontLeft, kFrontRight, kBackLeft, kBackRight);

    private static final TunableDouble kTurboSpin = Tunable.doubleValue("swerve/kTurboSpin", 8.0);
    private static final TunableDouble kInLineTolerance = Tunable.doubleValue("swerve/kInLineTolerance", 0.35);

    private static final TunableDouble kBeachSpeed = Tunable.doubleValue("swerve/beach/speed", 3.0);
    private static final TunableDouble kBeachTolerance = Tunable.doubleValue("swerve/beach/tolerance", 0.15);

    private static final TunableDouble kRepulsorX = Tunable.doubleValue("swerve/repulsor/x", 1.05);
    private static final TunableDouble kRepulsorLead = Tunable.doubleValue("swerve/repulsor/leadDistance", 0.78);
    private static final TunableDouble kRepulsorVelocity = Tunable.doubleValue("swerve/repulsor/velocity", 3.4);
    private static final TunableDouble kRepulsorSlowLead = Tunable.doubleValue("swerve/repulsor/slowdownLead", 0.71);
    private static final TunableDouble kRepulsorSlowL4Lead = Tunable.doubleValue("swerve/repulsor/slowdownL4Lead", 1.2);
    private static final TunableDouble kRepulsorSlowScore = Tunable.doubleValue("swerve/repulsor/slowdownScore", 1.25);
    private static final TunableDouble kRepulsorTolerance = Tunable.doubleValue("swerve/repulsor/tolerance", 0.2);
    private static final TunableDouble kRepulsorAngTolerance = Tunable.doubleValue("swerve/repulsor/angTolerance", 0.4);

    private static final TunableDouble kReefAssistX = Tunable.doubleValue("swerve/reefAssist/x", 0.681);
    private static final TunableDouble kReefAssistKp = Tunable.doubleValue("swerve/reefAssist/kP", 20.0);
    private static final TunableDouble kReefAssistTolerance = Tunable.doubleValue("swerve/reefAssist/tolerance", 1.75);

    private static final TunableDouble kFacingReefTolerance = Tunable.doubleValue("swerve/kFacingReefTolerance", 1.0);
    private static final TunableDouble kReefDangerDistance = Tunable.doubleValue("swerve/kReefDangerDistance", 0.7);
    private static final TunableDouble kReefHappyDistance = Tunable.doubleValue("swerve/kReefHappyDistance", 3.25);
    private static final TunableDouble kGoosingDistance = Tunable.doubleValue("swerve/kGoosingDistance", 0.95);

    private final SwerveAPI api;
    private final SwerveState state;
    private final VisionManager vision;
    private final RepulsorField repulsor;

    private final PIDController autoPIDx;
    private final PIDController autoPIDy;
    private final PIDController autoPIDangular;

    private final ProfiledPIDController angularPID;

    private final ReefAssistData reefAssist = new ReefAssistData();

    @SuppressWarnings("unused")
    private Pose2d autoLast = null;

    private Pose2d autoNext = null;
    private Pose2d reefReference = Pose2d.kZero;
    private boolean facingReef = false;
    private double wallDistance = 0.0;

    public Swerve() {
        api = new SwerveAPI(kConfig);
        state = api.state;
        vision = VisionManager.getInstance();
        repulsor = new RepulsorField(TimedRobot.kDefaultPeriod, FieldConstants.kObstacles);

        autoPIDx = new PIDController(10.0, 0.0, 0.0);
        autoPIDy = new PIDController(10.0, 0.0, 0.0);
        autoPIDangular = new PIDController(10.0, 0.0, 0.0);
        autoPIDangular.enableContinuousInput(-Math.PI, Math.PI);

        angularPID = new ProfiledPIDController(10.0, 0.5, 0.25, new Constraints(10.0, 24.0));
        angularPID.enableContinuousInput(-Math.PI, Math.PI);
        angularPID.setIZone(0.8);

        api.enableTunables("swerve/api");
        Tunable.pidController("swerve/autoPID", autoPIDx);
        Tunable.pidController("swerve/autoPID", autoPIDy);
        Tunable.pidController("swerve/autoPIDangular", autoPIDangular);
        Tunable.pidController("swerve/angularPID", angularPID);
    }

    @Override
    public void periodic() {
        Profiler.start("Swerve.periodic()");

        // Refresh the swerve API.
        Profiler.run("SwerveAPI.refresh()", api::refresh);

        // Apply vision estimates to the pose estimator.
        Profiler.run("SwerveAPI.addVisionMeasurements()", () ->
            api.addVisionMeasurements(vision.getUnreadResults(state.poseHistory))
        );

        // Calculate helpers
        Translation2d reefCenter = Alliance.isBlue() ? FieldConstants.kReefCenterBlue : FieldConstants.kReefCenterRed;
        Translation2d reefTranslation = state.translation.minus(reefCenter);
        Rotation2d reefAngle = new Rotation2d(
            Math.floor(
                reefCenter.minus(state.translation).getAngle().plus(new Rotation2d(Math2.kSixthPi)).getRadians() /
                Math2.kThirdPi
            ) *
            Math2.kThirdPi
        );

        // Save the current alliance's reef location, and the rotation
        // to the reef wall relevant to the robot's position.
        reefReference = new Pose2d(reefCenter, reefAngle);

        // If the robot is rotated to face the reef, within an arbitrary tolerance.
        facingReef = Math2.epsilonEquals(
            0.0,
            reefAngle.minus(state.rotation).getRadians(),
            kFacingReefTolerance.value()
        );

        // Calculate the distance from the robot's center to the nearest reef wall face.
        wallDistance = Math.max(
            0,
            reefAngle.rotateBy(Rotation2d.kPi).minus(reefTranslation.getAngle()).getCos() * reefTranslation.getNorm() -
            FieldConstants.kReefCenterToWallDistance
        );

        Profiler.end();
    }

    /**
     * Returns the current blue origin relative pose of the robot.
     */
    @NotLogged
    public Pose2d getPose() {
        return state.pose;
    }

    /**
     * Returns the directionless measured velocity of the robot, in m/s.
     */
    @NotLogged
    public double getVelocity() {
        return state.velocity;
    }

    /**
     * Returns true if the goose is happy!!
     * (Robot is facing the reef and within the happy distance).
     */
    public boolean happyGoose() {
        return (
            facingReef &&
            wallDistance < kReefHappyDistance.value() &&
            Math.abs(reefAssist.error) < kInLineTolerance.value()
        );
    }

    /**
     * Returns true if the elevator and goose neck are safe
     * to move, based on the robot's position on the field.
     */
    public boolean wildlifeConservationProgram() {
        return wallDistance > kReefDangerDistance.value();
    }

    public boolean goosingTime() {
        return wallDistance < kGoosingDistance.value();
    }

    /**
     * Tares the rotation of the robot. Useful for
     * fixing an out of sync or drifting IMU.
     */
    public Command tareRotation() {
        return commandBuilder("Swerve.tareRotation()")
            .onInitialize(() -> {
                api.tareRotation(Perspective.kOperator);
                vision.reset();
            })
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
        return commandBuilder("Swerve.drive()").onExecute(() -> {
            double pitch = state.pitch.getRadians();
            double roll = state.roll.getRadians();

            var antiBeach = Perspective.kOperator.toPerspectiveSpeeds(
                new ChassisSpeeds(
                    Math.abs(pitch) > kBeachTolerance.value() ? Math.copySign(kBeachSpeed.value(), pitch) : 0.0,
                    Math.abs(roll) > kBeachTolerance.value() ? Math.copySign(kBeachSpeed.value(), -roll) : 0.0,
                    0.0
                ),
                state.rotation
            );

            api.applyAssistedDriverInput(
                x.getAsDouble(),
                y.getAsDouble(),
                angular.getAsDouble(),
                antiBeach,
                Perspective.kOperator,
                true,
                true
            );
        });
    }

    /**
     * SPIN FAST RAHHHHHHH
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     * @param angular The CCW+ angular speed to apply, from {@code [-1.0, 1.0]}.
     */
    public Command turboSpin(DoubleSupplier x, DoubleSupplier y, DoubleSupplier angular) {
        Mutable<Double> configured = new Mutable<>(0.0);
        return drive(x, y, angular)
            .beforeStarting(() -> {
                configured.value = api.config.driverAngularVel;
                api.config.driverAngularVel = kTurboSpin.value();
            })
            .finallyDo(() -> api.config.driverAngularVel = configured.value);
    }

    /**
     * Drives the robot using driver input while facing the reef,
     * and "pushing" the robot to center on the selected pipe.
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     * @param angular The CCW+ angular speed to apply, from {@code [-1.0, 1.0]}.
     * @param left A supplier that returns {@code true} if the robot should target
     *             the left reef pole, or {@code false} to target the right pole.
     */
    public Command driveReef(DoubleSupplier x, DoubleSupplier y, DoubleSupplier angular, BooleanSupplier left) {
        Mutable<Boolean> exitLock = new Mutable<>(false);

        return commandBuilder("Swerve.driveReef()")
            .onInitialize(() -> {
                angularPID.reset(state.rotation.getRadians(), state.speeds.omegaRadiansPerSecond);
                exitLock.value = false;
            })
            .onExecute(() -> {
                double xInput = x.getAsDouble();
                double yInput = y.getAsDouble();
                double angularInput = angular.getAsDouble();
                double norm = Math.hypot(-yInput, -xInput);
                boolean inDeadband = norm < api.config.driverVelDeadband;

                reefAssist.targetPipe = generateReefLocation(
                    kReefAssistX.value(),
                    reefReference.getRotation(),
                    left.getAsBoolean()
                );

                Rotation2d robotAngle = reefAssist.targetPipe.getTranslation().minus(state.translation).getAngle();
                Rotation2d xyAngle = !inDeadband
                    ? new Rotation2d(-yInput, -xInput).rotateBy(Alliance.isBlue() ? Rotation2d.kZero : Rotation2d.kPi)
                    : robotAngle;

                double stickDistance = Math.abs(
                    (Math.cos(xyAngle.getRadians()) * (reefAssist.targetPipe.getY() - state.pose.getY()) -
                        Math.sin(xyAngle.getRadians()) * (reefAssist.targetPipe.getX() - state.pose.getX()))
                );

                reefAssist.running =
                    Math2.epsilonEquals(0.0, stickDistance, kReefAssistTolerance.value()) &&
                    Math2.epsilonEquals(0.0, robotAngle.minus(xyAngle).getRadians(), Math2.kHalfPi) &&
                    !inDeadband;

                reefAssist.error = robotAngle.minus(reefReference.getRotation()).getRadians();
                reefAssist.output = reefAssist.running ? reefAssist.error * norm * norm * kReefAssistKp.value() : 0.0;

                var assist = Perspective.kOperator.toPerspectiveSpeeds(
                    new ChassisSpeeds(
                        0.0,
                        reefAssist.output,
                        !exitLock.value
                            ? angularPID.calculate(
                                state.rotation.getRadians(),
                                reefReference.getRotation().getRadians()
                            )
                            : 0.0
                    ),
                    reefReference.getRotation()
                );

                if (!Math2.epsilonEquals(angularInput, 0.0)) exitLock.value = true;
                api.applyAssistedDriverInput(xInput, yInput, angularInput, assist, Perspective.kOperator, true, true);
            })
            .onEnd(() -> reefAssist.running = false);
    }

    /**
     * Drives the robot to the reef autonomously. Targets
     * the side of the reef that the robot is closest to.
     * @param left A supplier that returns {@code true} if the robot should target
     *             the left reef pole, or {@code false} to target the right pole.
     * @param ready If the robot is ready to approach the scoring location.
     * @param l4 If the robot is scoring L4.
     */
    public Command repulsorDrive(BooleanSupplier left, BooleanSupplier ready, BooleanSupplier l4) {
        return repulsorDrive(() -> reefReference.getRotation(), left, ready, l4);
    }

    /**
     * Drives the robot to the reef autonomously.
     * @param location The reef location to drive to.
     * @param ready If the robot is ready to approach the scoring location.
     * @param l4 If the robot is scoring L4.
     */
    public Command repulsorDrive(ReefLocation location, BooleanSupplier ready, BooleanSupplier l4) {
        return repulsorDrive(
            () -> Alliance.isBlue() ? location.side : location.side.rotateBy(Rotation2d.kPi),
            () -> location.left,
            ready,
            l4
        );
    }

    /**
     * Internal function, converts reef side to repulsor drive controller.
     * @param side A supplier that returns the side of the reef to target.
     * @param left A supplier that returns {@code true} if the robot should target
     *             the left reef pole, or {@code false} to target the right pole.
     * @param ready If the robot is ready to approach the scoring location.
     * @param l4 If the robot is scoring L4.
     */
    private Command repulsorDrive(
        Supplier<Rotation2d> side,
        BooleanSupplier left,
        BooleanSupplier ready,
        BooleanSupplier l4
    ) {
        Mutable<Pose2d> lastTarget = new Mutable<>(Pose2d.kZero);
        Mutable<Boolean> nowSafe = new Mutable<>(false);

        return repulsorDrive(
            () -> {
                Pose2d target =
                    reefAssist.targetPipe = generateReefLocation(kRepulsorX.value(), side.get(), left.getAsBoolean());

                Rotation2d robotAngle = target.getTranslation().minus(state.translation).getAngle();
                reefAssist.error = robotAngle.minus(side.get()).getRadians();

                if (!target.equals(lastTarget.value)) nowSafe.value = false;
                lastTarget.value = target;

                if (!nowSafe.value) {
                    target = generateReefLocation(
                        kRepulsorX.value() + kRepulsorLead.value(),
                        side.get(),
                        left.getAsBoolean()
                    );

                    if (
                        ready.getAsBoolean() &&
                        state.translation.getDistance(target.getTranslation()) *
                        (Math.abs(reefAssist.error) / Math.PI) <=
                        kRepulsorTolerance.value() &&
                        Math.abs(state.rotation.minus(target.getRotation()).getRadians()) <=
                        kRepulsorAngTolerance.value()
                    ) {
                        nowSafe.value = true;
                    }
                }

                return target;
            },
            () ->
                !nowSafe.value
                    ? (l4.getAsBoolean() ? kRepulsorSlowL4Lead.value() : kRepulsorSlowLead.value())
                    : kRepulsorSlowScore.value()
        ).beforeStarting(() -> {
            lastTarget.value = Pose2d.kZero;
            nowSafe.value = false;
        });
    }

    /**
     * Drives the robot to a target position using the repulsor field, ending
     * when the robot is within a specified tolerance of the target.
     * @param target A supplier that returns the target blue-origin relative field location.
     * @param slowdownRange A supplier that returns the range in meters at which to begin slowing down the robot.
     * @param endTolerance The tolerance in meters at which to end the command.
     */
    public Command repulsorDrive(Supplier<Pose2d> target, DoubleSupplier slowdownRange, DoubleSupplier endTolerance) {
        return repulsorDrive(target, slowdownRange).until(
            () -> target.get().getTranslation().getDistance(state.translation) < endTolerance.getAsDouble()
        );
    }

    /**
     * Drives the robot to a target position using the repulsor field.
     * @param target A supplier that returns the target blue-origin relative field location.
     * @param slowdownRange A supplier that returns the range in meters at which to begin slowing down the robot.
     */
    public Command repulsorDrive(Supplier<Pose2d> target, DoubleSupplier slowdownRange) {
        return commandBuilder("Swerve.repulsorDrive()")
            .onInitialize(() -> angularPID.reset(state.rotation.getRadians(), state.speeds.omegaRadiansPerSecond))
            .onExecute(() -> {
                repulsor.setTarget(target.get());
                RepulsorSample sample = repulsor.sampleField(
                    state.translation,
                    kRepulsorVelocity.value(),
                    slowdownRange.getAsDouble()
                );

                api.applySpeeds(
                    new ChassisSpeeds(
                        sample.vx(),
                        sample.vy(),
                        angularPID.calculate(
                            state.rotation.getRadians(),
                            repulsor.getTarget().getRotation().getRadians()
                        )
                    ),
                    Perspective.kBlue,
                    true,
                    true
                );
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
     * Resets the pose of the robot, inherently seeding field-relative movement. This
     * method is not intended for use outside of creating an {@link AutoFactory}.
     * @param pose The new blue origin relative pose to apply to the pose estimator.
     */
    public void resetPose(Pose2d pose) {
        api.resetPose(pose);
        vision.reset();

        autoPIDx.reset();
        autoPIDy.reset();
        autoPIDangular.reset();
    }

    /**
     * Follows a Choreo trajectory by moving towards the next sample. This method
     * is not intended for use outside of creating an {@link AutoFactory}.
     * @param sample The next trajectory sample.
     */
    public void followTrajectory(SwerveSample sample) {
        autoLast = autoNext;
        autoNext = sample.getPose();

        Pose2d pose = state.pose;
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

    private Pose2d generateReefLocation(double xOffset, Rotation2d side, boolean left) {
        return new Pose2d(
            reefReference
                .getTranslation()
                .plus(new Translation2d(-xOffset, FieldConstants.kPipeOffsetY * (left ? 1.0 : -1.0)).rotateBy(side)),
            side
        );
    }

    @Logged
    public final class ReefAssistData {

        private Pose2d targetPipe = Pose2d.kZero;
        private boolean running = false;
        private double error = 0.0;
        private double output = 0.0;
    }
}
