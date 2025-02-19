package org.team340.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.NotifierCommand;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.team340.lib.util.Math2;
import org.team340.lib.util.Mutable;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.lib.util.command.CommandBuilder;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.UpperCAN;

@Logged
public class GooseNeck extends GRRSubsystem {

    /**
     * A position for the goose neck. All positions in this enum should be
     * a positive value, moving the goose neck to the *right* of the robot
     * (CCW+). Moving the goose neck to the left should be specified in
     * method arguments, with the applied rotation being inverted from
     * the values specified here.
     */
    private static enum GoosePosition {
        kIn(0.0),
        kScoreL1(-0.12),
        kScoreForward(0.5);

        private TunableDouble rotations;

        private GoosePosition(double rotations) {
            this.rotations = Tunable.doubleValue("gooseNeck/positions/" + name(), rotations);
        }

        private double rotations() {
            return rotations.value();
        }

        public static GoosePosition closeTo(double position) {
            for (GoosePosition option : values()) {
                if (Math2.epsilonEquals(option.rotations(), position, kCloseToTolerance.value())) {
                    return option;
                }
            }

            return null;
        }
    }

    private static enum GooseSpeed {
        kIntakeFast(-4.0),
        kIntakeSlow(-2.0),
        kScoreL1(-3.5),
        kScoreForward(10.0),
        kBarf(-8.0),
        kSwallow(8.0);

        private TunableDouble voltage;

        private GooseSpeed(double voltage) {
            this.voltage = Tunable.doubleValue("gooseNeck/speeds/" + name(), voltage);
        }

        private double voltage() {
            return voltage.value();
        }
    }

    private static final TunableDouble kCloseToTolerance = Tunable.doubleValue("gooseNeck/kCloseToTolerance", 0.08);
    private static final TunableDouble kAtPositionEpsilon = Tunable.doubleValue("gooseNeck/kAtPositionEpsilon", 0.01);

    private static final TunableDouble kIntakeSeatDelay = Tunable.doubleValue("gooseNeck/kIntakeSeatDelay", 0.02);
    private static final TunableDouble kTorqueDelay = Tunable.doubleValue("gooseNeck/kTorqueDelay", 0.2);
    private static final TunableDouble kTorqueMax = Tunable.doubleValue("gooseNeck/kTorqueMax", 10.0);

    private final TalonFX pivotMotor;
    private final TalonFXS beakMotor;
    private final CANdi candi;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Boolean> beamBreak;
    private final StatusSignal<Boolean> beamBreakVolatile;

    private final MotionMagicVoltage positionControl;
    private final TorqueCurrentFOC torqueControl;
    private final VoltageOut beakVoltageControl;

    private final PIDController torquePID = new PIDController(165.0, 0.0, 12.0);
    private final AtomicBoolean hasCoral = new AtomicBoolean(false);
    private boolean goosing = false;

    public GooseNeck() {
        pivotMotor = new TalonFX(UpperCAN.kGooseNeckMotor);
        beakMotor = new TalonFXS(UpperCAN.kGooseBeakMotor);
        candi = new CANdi(UpperCAN.kGooseCANdi);

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

        pivotConfig.CurrentLimits.StatorCurrentLimit = 60.0;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = 40.0;

        pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        pivotConfig.Feedback.SensorToMechanismRatio = (50.0 / 10.0) * (72.0 / 14.0);

        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 20.0;
        pivotConfig.MotionMagic.MotionMagicAcceleration = 20.0;

        pivotConfig.Slot0.kP = 60.0;
        pivotConfig.Slot0.kI = 0.0;
        pivotConfig.Slot0.kD = 0.0;
        pivotConfig.Slot0.kS = 0.0;
        pivotConfig.Slot0.kV = 2.5;

        pivotConfig.TorqueCurrent.PeakForwardTorqueCurrent = pivotConfig.CurrentLimits.StatorCurrentLimit;
        pivotConfig.TorqueCurrent.PeakReverseTorqueCurrent = -pivotConfig.CurrentLimits.StatorCurrentLimit;

        TalonFXSConfiguration beakConfig = new TalonFXSConfiguration();

        beakConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        beakConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        beakConfig.CurrentLimits.SupplyCurrentLimit = 30.0;

        beakConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        beakConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        CANdiConfiguration candiConfig = new CANdiConfiguration();
        candiConfig.PWM2.AbsoluteSensorOffset = 0.925;
        candiConfig.PWM2.SensorDirection = true;

        PhoenixUtil.run("Clear Goose Neck Pivot Sticky Faults", () -> pivotMotor.clearStickyFaults());
        PhoenixUtil.run("Clear Goose Neck Beak Sticky Faults", () -> beakMotor.clearStickyFaults());
        PhoenixUtil.run("Clear Goose Neck CANdi Sticky Faults", () -> candi.clearStickyFaults());
        PhoenixUtil.run("Apply Goose Neck TalonFXConfiguration", () -> pivotMotor.getConfigurator().apply(pivotConfig));
        PhoenixUtil.run("Apply Goose Neck Beak TalonFXSConfiguration", () ->
            beakMotor.getConfigurator().apply(beakConfig)
        );
        PhoenixUtil.run("Apply Goose Neck CANdiConfiguration", () -> candi.getConfigurator().apply(candiConfig));

        position = pivotMotor.getPosition();
        velocity = pivotMotor.getVelocity();
        beamBreak = candi.getS1Closed();
        beamBreakVolatile = candi.getS1Closed().clone();

        PhoenixUtil.run("Set Goose Neck Signal Frequencies", () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                position,
                velocity,
                candi.getPWM2Position(),
                candi.getPWM2Velocity()
            )
        );
        PhoenixUtil.run("Set Goose Neck Fast Signal Frequencies", () ->
            BaseStatusSignal.setUpdateFrequencyForAll(250, beamBreak, beamBreakVolatile)
        );
        PhoenixUtil.run("Optimize Goose Neck CAN Utilization", () ->
            ParentDevice.optimizeBusUtilizationForAll(5, pivotMotor, beakMotor, candi)
        );

        positionControl = new MotionMagicVoltage(0.0);
        torqueControl = new TorqueCurrentFOC(0.0);
        beakVoltageControl = new VoltageOut(0.0);
        beakVoltageControl.EnableFOC = false;
        beakVoltageControl.UpdateFreqHz = 0.0;

        Tunable.pidController("gooseNeck/pid", pivotMotor);
        Tunable.motionProfile("gooseNeck/motion", pivotMotor);
        Tunable.pidController("gooseNeck/torquePID", torquePID);

        PhoenixUtil.run("Sync Goose Neck Position", () ->
            pivotMotor.setPosition(
                MathUtil.inputModulus(candi.getPWM2Position(false).waitForUpdate(1.0).getValueAsDouble(), -0.5, 0.5)
            )
        );
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(position, velocity, beamBreak);
    }

    // *************** Helper Functions ***************

    @NotLogged
    public boolean hasCoral() {
        return hasCoral.get();
    }

    @NotLogged
    public boolean goosing() {
        return goosing;
    }

    public double getPosition() {
        return BaseStatusSignal.getLatencyCompensatedValueAsDouble(position, velocity);
    }

    /**
     * Checks if the beam break detects an object.
     * @return True if the beam break detects an object, false otherwise.
     */
    public boolean beamBroken() {
        return !beamBreak.getValue();
    }

    // *************** Commands ***************

    public Command stow(BooleanSupplier safe) {
        return goTo(() -> GoosePosition.kIn, safe, false, false).withName("GooseNeck.stow()");
    }

    public Command intake(BooleanSupplier button, BooleanSupplier safe) {
        Mutable<Boolean> sawCoral = new Mutable<>(false);
        Timer delay = new Timer();

        return goTo(() -> GoosePosition.kIn, safe, false, false).withDeadline(
            new NotifierCommand(
                () -> {
                    boolean beamBroken = !beamBreakVolatile.waitForUpdate(0.02).getValue();

                    if (beamBroken) sawCoral.value = true;
                    if (sawCoral.value && !beamBroken) delay.start();

                    if (!sawCoral.value) {
                        beakMotor.setControl(beakVoltageControl.withOutput(GooseSpeed.kIntakeFast.voltage()));
                    } else if (!delay.hasElapsed(kIntakeSeatDelay.value())) {
                        beakMotor.setControl(beakVoltageControl.withOutput(GooseSpeed.kIntakeSlow.voltage()));
                    } else {
                        hasCoral.set(true);
                        beakMotor.stopMotor();
                    }
                },
                0.0
            )
                .until(() -> hasCoral() || (!button.getAsBoolean() && !sawCoral.value))
                .beforeStarting(() -> {
                    sawCoral.value = false;
                    delay.stop();
                    delay.reset();
                })
                .finallyDo(beakMotor::stopMotor)
                .onlyIf(() -> !hasCoral())
                .withName("GooseNeck.intake()")
        );
    }

    public Command score(
        BooleanSupplier runManual,
        BooleanSupplier l1,
        BooleanSupplier safe,
        boolean left,
        boolean allowMovement
    ) {
        Mutable<Boolean> beamTrigger = new Mutable<>(false);

        return goTo(
            () -> l1.getAsBoolean() ? GoosePosition.kScoreL1 : GoosePosition.kScoreForward,
            safe,
            left,
            allowMovement
        )
            .alongWith(
                new CommandBuilder()
                    .onInitialize(() -> beamTrigger.value = false)
                    .onExecute(() -> {
                        if (beamBroken() || beamTrigger.value || runManual.getAsBoolean()) {
                            beakMotor.setControl(
                                beakVoltageControl.withOutput(
                                    l1.getAsBoolean()
                                        ? GooseSpeed.kScoreL1.voltage()
                                        : GooseSpeed.kScoreForward.voltage()
                                )
                            );
                            hasCoral.set(false);
                            beamTrigger.value = true;
                        }
                    })
                    .onEnd(beakMotor::stopMotor)
            )
            .onlyIf(this::hasCoral)
            .withName("GooseNeck.score(" + left + ")");
    }

    public Command barf(BooleanSupplier safe) {
        return goTo(() -> GoosePosition.kIn, safe, false, false)
            .alongWith(
                new CommandBuilder()
                    .onInitialize(() -> hasCoral.set(false))
                    .onExecute(() -> beakMotor.setControl(beakVoltageControl.withOutput(GooseSpeed.kBarf.voltage())))
                    .onEnd(beakMotor::stopMotor)
            )
            .withName("GooseNeck.barf()");
    }

    public Command swallow(BooleanSupplier safe) {
        return goTo(() -> GoosePosition.kIn, safe, false, false)
            .alongWith(
                new CommandBuilder()
                    .onInitialize(() -> hasCoral.set(false))
                    .onExecute(() -> beakMotor.setControl(beakVoltageControl.withOutput(GooseSpeed.kSwallow.voltage())))
                    .onEnd(beakMotor::stopMotor)
            )
            .withName("GooseNeck.swallow()");
    }

    /**
     * Moves the pivot to predetermined positions.
     * @param position The position to move the pivot to.
     */
    private Command goTo(Supplier<GoosePosition> position, BooleanSupplier safe, boolean left, boolean allowMovement) {
        Mutable<Double> holdPosition = new Mutable<>(-1.0);
        Mutable<Double> lastTarget = new Mutable<>(0.0);
        Timer timer = new Timer();

        return commandBuilder("GooseNeck.goTo(" + left + ", " + allowMovement + ")")
            .onInitialize(() -> {
                timer.stop();
                timer.reset();
                torquePID.reset();
            })
            .onExecute(() -> {
                double target = position.get().rotations() * (left ? -1.0 : 1.0);
                if (lastTarget.value != target) {
                    timer.stop();
                    timer.reset();
                }

                lastTarget.value = target;

                if (allowMovement && position.get().equals(GoosePosition.kScoreForward)) {
                    double currentPosition = getPosition();
                    boolean atPosition = Math2.epsilonEquals(currentPosition, target, kAtPositionEpsilon.value());
                    goosing = true;

                    if (timer.hasElapsed(kTorqueDelay.value())) {
                        if (atPosition) {
                            pivotMotor.stopMotor();
                        } else {
                            pivotMotor.setControl(
                                torqueControl.withOutput(
                                    MathUtil.clamp(
                                        torquePID.calculate(currentPosition, target),
                                        -kTorqueMax.value(),
                                        kTorqueMax.value()
                                    )
                                )
                            );
                        }

                        return;
                    } else if (atPosition) {
                        timer.start();
                    }
                } else {
                    goosing = false;
                }

                if (!safe.getAsBoolean()) {
                    if (holdPosition.value < 0.0) {
                        double currentPosition = getPosition();
                        GoosePosition close = GoosePosition.closeTo(currentPosition * (left ? -1.0 : 1.0));
                        holdPosition.value = close != null ? close.rotations() * (left ? -1.0 : 1.0) : currentPosition;
                    }
                    pivotMotor.setControl(positionControl.withPosition(holdPosition.value));
                } else {
                    pivotMotor.setControl(positionControl.withPosition(target));
                    holdPosition.value = -1.0;
                }
            })
            .onEnd(() -> {
                pivotMotor.stopMotor();
                goosing = false;
            });
    }
}
