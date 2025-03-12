package org.team340.robot.subsystems;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
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
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.robot.Constants;
import org.team340.robot.Constants.FieldConstants;
import org.team340.robot.Constants.LowerCAN;
import org.team340.robot.util.VisionManager;

/**
 * The robot's swerve drivetrain.
 */
@Logged
public final class Swerve extends GRRSubsystem {

    private static final double kMoveRatio = (54.0 / 10.0) * (18.0 / 38.0) * (45.0 / 15.0);
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
        .setTimings(TimedRobot.kDefaultPeriod, 0.004, 0.02)
        .setMovePID(0.27, 0.0, 0.0)
        .setMoveFF(0.0, 0.126)
        .setTurnPID(100.0, 0.0, 0.2)
        .setBrakeMode(false, true)
        .setLimits(4.0, 0.05, 17.5, 14.0, 30.0)
        .setDriverProfile(4.0, 1.5, 0.15, 4.7, 2.0, 0.05)
        .setPowerProperties(Constants.kVoltage, 100.0, 80.0, 60.0, 60.0)
        .setMechanicalProperties(kMoveRatio, kTurnRatio, 0.0, Units.inchesToMeters(4.0))
        .setOdometryStd(0.1, 0.1, 0.1)
        .setIMU(SwerveIMUs.canandgyro(LowerCAN.kCanandgyro))
        .setPhoenixFeatures(new CANBus(LowerCAN.kLowerCANBus), true, true, true)
        .setModules(kFrontLeft, kFrontRight, kBackLeft, kBackRight);

    private static final TunableDouble kTurboSpin = Tunable.doubleValue("swerve/kTurboSpin", 8.0);

    private static final TunableDouble kBeachSpeed = Tunable.doubleValue("swerve/kBeachSpeed", 3.0);
    private static final TunableDouble kBeachTolerance = Tunable.doubleValue("swerve/kBeachTolerance", 0.15);

    private static final TunableDouble kReefAssistKp = Tunable.doubleValue("swerve/kReefAssistKp", 15.0);
    private static final TunableDouble kReefAssistTolerance = Tunable.doubleValue("swerve/kReefAssistTolerance", 1.3);
    private static final TunableDouble kFacingReefTolerance = Tunable.doubleValue("swerve/kFacingReefTolerance", 1.0);
    private static final TunableDouble kReefDangerDistance = Tunable.doubleValue("swerve/kReefDangerDistance", 0.6);
    private static final TunableDouble kReefHappyDistance = Tunable.doubleValue("swerve/kReefHappyDistance", 3.0);

    private final SwerveAPI api;
    private final SwerveState state;
    private final VisionManager vision;

    private final PIDController autoPIDx;
    private final PIDController autoPIDy;
    private final PIDController autoPIDangular;

    private final ProfiledPIDController angularPID;

    private final Debouncer dangerDebounce = new Debouncer(0.2);
    private final ReefAssistData reefAssist = new ReefAssistData();

    private Pose2d autoLast = null;
    private Pose2d autoNext = null;
    private Pose2d reefReference = Pose2d.kZero;
    private boolean facingReef = false;
    private double wallDistance = 0.0;

    public Swerve() {
        api = new SwerveAPI(kConfig);
        state = api.state;
        vision = VisionManager.getInstance();

        autoPIDx = new PIDController(10.0, 0.0, 0.0);
        autoPIDy = new PIDController(10.0, 0.0, 0.0);
        autoPIDangular = new PIDController(10.0, 0.0, 0.0);
        autoPIDangular.enableContinuousInput(-Math.PI, Math.PI);

        angularPID = new ProfiledPIDController(10.0, 0.5, 0.25, new Constraints(10.0, 30.0));
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
        // Refresh the swerve API.
        api.refresh();

        // Apply vision estimates to the pose estimator.
        api.addVisionMeasurements(vision.getUnreadResults(state.imu.yawMeasurements));

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
        return facingReef && wallDistance < kReefHappyDistance.value();
    }

    /**
     * Returns true if the elevator and goose neck are safe
     * to move, based on the robot's position on the field.
     */
    public boolean wildlifeConservationProgram() {
        return dangerDebounce.calculate(wallDistance > kReefDangerDistance.value());
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
        return commandBuilder("Swerve.drive()").onExecute(() -> {
            double pitch = state.imu.pitch.getRadians();
            double roll = state.imu.roll.getRadians();

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

                reefAssist.targetPipe = new Pose2d(
                    reefReference
                        .getTranslation()
                        .plus(
                            new Translation2d(
                                FieldConstants.kPipeOffsetX,
                                FieldConstants.kPipeOffsetY * (left.getAsBoolean() ? -1.0 : 1.0)
                            ).rotateBy(reefReference.getRotation())
                        ),
                    reefReference.getRotation()
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
     * Drives the modules to stop the robot from moving.
     * @param lock If the wheels should be driven to an X formation to stop the robot from being pushed.
     */
    public Command stop(boolean lock) {
        return commandBuilder("Swerve.stop(" + lock + ")").onExecute(() -> api.applyStop(lock));
    }

    /**
     * Resets the autonomous trajectory following PID controllers. This
     * command does not require the swerve subsystem, and can be safely
     * composed in parallel with another swerve command.
     */
    public Command resetAutoPID() {
        return Commands.runOnce(() -> {
            autoLast = null;
            autoNext = autoLast;
            autoPIDx.reset();
            autoPIDy.reset();
            autoPIDangular.reset();
        })
            .ignoringDisable(true)
            .withName("Swerve.resetAutoPID");
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

    @Logged
    public final class ReefAssistData {

        private Pose2d targetPipe = Pose2d.kZero;
        private boolean running = false;
        private double error = 0.0;
        private double output = 0.0;
    }
}
