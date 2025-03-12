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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
import org.team340.robot.util.ReefSelection;

@Logged
public final class GooseNeck extends GRRSubsystem {

    /**
     * A position for the goose neck. All positions in this enum should be
     * a positive value, moving the goose neck to the *right* of the robot
     * (CCW+). Moving the goose neck to the left should be specified in
     * method arguments, with the applied rotation being inverted from
     * the values specified here.
     */
    private static enum GoosePosition {
        kStow(0.0),
        kScoreL1(-0.1),
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
        kIntake(-6.0),
        kSeat(-2.3),
        kScoreL1(-3.5),
        kAlgae(12.0),
        kScoreForward(9.0),
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
    private static final TunableDouble kAtPositionTolerance = Tunable.doubleValue(
        "gooseNeck/kAtPositionTolerance",
        0.01
    );
    private static final TunableDouble kStickyScoreTolerance = Tunable.doubleValue(
        "gooseNeck/kStickyScoreTolerance",
        0.2
    );

    private static final TunableDouble kFindEdgeDelay = Tunable.doubleValue("gooseNeck/kFindEdgeDelay", 0.062);
    private static final TunableDouble kSeatDelay = Tunable.doubleValue("gooseNeck/kSeatDelay", 0.03);
    private static final TunableDouble kTorqueDelay = Tunable.doubleValue("gooseNeck/kTorqueDelay", 0.2);
    private static final TunableDouble kTorqueMax = Tunable.doubleValue("gooseNeck/kTorqueMax", 12.0);

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

    private final PIDController torquePID = new PIDController(185.0, 0.0, 18.0);
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

        pivotConfig.Slot0.kP = 200.0;
        pivotConfig.Slot0.kI = 0.0;
        pivotConfig.Slot0.kD = 2.0;
        pivotConfig.Slot0.kS = 0.0;
        pivotConfig.Slot0.kV = 2.35;

        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.0;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -1.0;

        pivotConfig.TorqueCurrent.PeakForwardTorqueCurrent = pivotConfig.CurrentLimits.StatorCurrentLimit;
        pivotConfig.TorqueCurrent.PeakReverseTorqueCurrent = -pivotConfig.CurrentLimits.StatorCurrentLimit;

        TalonFXSConfiguration beakConfig = new TalonFXSConfiguration();

        beakConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        beakConfig.CurrentLimits.StatorCurrentLimit = 50.0;
        beakConfig.CurrentLimits.SupplyCurrentLimit = 30.0;

        beakConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        beakConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        CANdiConfiguration candiConfig = new CANdiConfiguration();
        candiConfig.PWM2.AbsoluteSensorOffset = 0.92;
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
            BaseStatusSignal.setUpdateFrequencyForAll(500, beamBreak, beamBreakVolatile)
        );
        PhoenixUtil.run("Optimize Goose Neck CAN Utilization", () ->
            ParentDevice.optimizeBusUtilizationForAll(10, pivotMotor, beakMotor, candi)
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

        // Warm-up enums
        GoosePosition.kStow.rotations();
        GooseSpeed.kIntake.voltage();
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
    public boolean noCoral() {
        return !hasCoral();
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

    public Command setHasCoral(boolean hasCoral) {
        return runOnce(() -> this.hasCoral.set(hasCoral))
            .ignoringDisable(true)
            .withName("Intake.setHasCoral(" + hasCoral + ")");
    }

    public Command stow(BooleanSupplier safe) {
        return goTo(() -> GoosePosition.kStow, () -> false, () -> false, safe).withName("GooseNeck.stow()");
    }

    public Command intake(BooleanSupplier button, BooleanSupplier swallow, BooleanSupplier safe) {
        enum State {
            kInit,
            kSawCoral,
            kFindingFallingEdge,
            kSeating,
            kDone
        }

        Mutable<State> state = new Mutable<>(State.kInit);
        Debouncer debounce = new Debouncer(0.065, DebounceType.kRising);
        Timer delay = new Timer();

        return goTo(() -> GoosePosition.kStow, () -> false, () -> false, safe).withDeadline(
            new NotifierCommand(
                () -> {
                    if (swallow.getAsBoolean()) {
                        beakMotor.setControl(beakVoltageControl.withOutput(GooseSpeed.kSwallow.voltage()));
                        hasCoral.set(false);
                        state.value = State.kInit;
                        return;
                    }

                    boolean beamBroken = debounce.calculate(!beamBreakVolatile.waitForUpdate(0.02).getValue());

                    switch (state.value) {
                        case kInit:
                            if (!beamBroken) {
                                beakMotor.setControl(beakVoltageControl.withOutput(GooseSpeed.kIntake.voltage()));
                                break;
                            }

                            delay.restart();
                            state.value = State.kSawCoral;
                        // Fall-through

                        case kSawCoral:
                            if (!delay.hasElapsed(kFindEdgeDelay.value()) && beamBroken) {
                                beakMotor.setControl(beakVoltageControl.withOutput(GooseSpeed.kIntake.voltage()));
                                break;
                            }

                            state.value = State.kFindingFallingEdge;
                        // Fall-through

                        case kFindingFallingEdge:
                            if (beamBroken) {
                                beakMotor.setControl(beakVoltageControl.withOutput(GooseSpeed.kSeat.voltage()));
                                break;
                            }

                            state.value = State.kSeating;
                            delay.restart();
                        // Fall-through

                        case kSeating:
                            if (!delay.hasElapsed(kSeatDelay.value())) {
                                beakMotor.setControl(beakVoltageControl.withOutput(GooseSpeed.kSeat.voltage()));
                                break;
                            }

                            state.value = State.kDone;
                        // Fall-through

                        case kDone:
                            hasCoral.set(true);
                            beakMotor.stopMotor();
                            break;
                    }
                },
                0.0
            )
                .until(() -> hasCoral() || (!button.getAsBoolean() && state.value.equals(State.kInit)))
                .beforeStarting(() -> {
                    state.value = State.kInit;
                    debounce.calculate(false);
                })
                .finallyDo(beakMotor::stopMotor)
                .onlyIf(this::noCoral)
                .withName("GooseNeck.receive()")
        );
    }

    public Command babyBird(BooleanSupplier button, BooleanSupplier safe) {
        enum State {
            kInit,
            kFindingFallingEdge,
            kSeating,
            kDone
        }

        Mutable<State> state = new Mutable<>(State.kInit);

        Debouncer debounce = new Debouncer(0.018, DebounceType.kBoth);
        Timer delay = new Timer();

        return goTo(() -> GoosePosition.kStow, () -> false, () -> false, safe).withDeadline(
            new NotifierCommand(
                () -> {
                    boolean beamBroken = debounce.calculate(!beamBreakVolatile.waitForUpdate(0.02).getValue());

                    switch (state.value) {
                        case kInit:
                            if (!beamBroken) {
                                beakMotor.setControl(beakVoltageControl.withOutput(-GooseSpeed.kIntake.voltage()));
                                break;
                            }

                            state.value = State.kFindingFallingEdge;
                        // Fall-through

                        case kFindingFallingEdge:
                            if (beamBroken) {
                                beakMotor.setControl(beakVoltageControl.withOutput(GooseSpeed.kSeat.voltage()));
                                break;
                            }

                            state.value = State.kSeating;
                            delay.start();
                        // Fall-through

                        case kSeating:
                            if (!delay.hasElapsed(kSeatDelay.value())) {
                                beakMotor.setControl(beakVoltageControl.withOutput(GooseSpeed.kSeat.voltage()));
                                break;
                            }

                            state.value = State.kDone;
                        // Fall-through

                        case kDone:
                            hasCoral.set(true);
                            beakMotor.stopMotor();
                            break;
                    }
                },
                0.0
            )
                .until(() -> hasCoral() || (!button.getAsBoolean() && state.value.equals(State.kInit)))
                .beforeStarting(() -> {
                    state.value = State.kInit;
                    debounce.calculate(false);
                    delay.stop();
                    delay.reset();
                })
                .finallyDo(beakMotor::stopMotor)
                .onlyIf(this::noCoral)
                .withName("GooseNeck.babyBird()")
        );
    }

    public Command score(
        ReefSelection selection,
        BooleanSupplier runManual,
        BooleanSupplier allowGoosing,
        BooleanSupplier safe
    ) {
        Mutable<Boolean> beamTrigger = new Mutable<>(false);

        return goTo(
            () -> !selection.isL1() ? GoosePosition.kScoreForward : GoosePosition.kScoreL1,
            () -> !selection.isL1() && allowGoosing.getAsBoolean(),
            () -> {
                double currentPosition = getPosition();
                if (Math.abs(currentPosition) > kStickyScoreTolerance.value()) {
                    return currentPosition < 0.0;
                } else {
                    return selection.isLeft();
                }
            },
            () -> selection.isL1() || safe.getAsBoolean()
        )
            .alongWith(
                new CommandBuilder()
                    .onInitialize(() -> beamTrigger.value = false)
                    .onExecute(() -> {
                        if (beamBroken() || beamTrigger.value || runManual.getAsBoolean()) {
                            beakMotor.setControl(
                                beakVoltageControl.withOutput(
                                    !selection.isL1()
                                        ? (allowGoosing.getAsBoolean()
                                                ? GooseSpeed.kScoreForward.voltage()
                                                : GooseSpeed.kAlgae.voltage())
                                        : GooseSpeed.kScoreL1.voltage()
                                )
                            );
                            hasCoral.set(false);
                            beamTrigger.value = true;
                        }
                    })
                    .onEnd(beakMotor::stopMotor)
            )
            .withName("GooseNeck.score()");
    }

    public Command barf(BooleanSupplier safe) {
        return goTo(() -> GoosePosition.kStow, () -> false, () -> false, safe)
            .alongWith(
                new CommandBuilder()
                    .onInitialize(() -> hasCoral.set(false))
                    .onExecute(() -> beakMotor.setControl(beakVoltageControl.withOutput(GooseSpeed.kBarf.voltage())))
                    .onEnd(beakMotor::stopMotor)
            )
            .withName("GooseNeck.barf()");
    }

    public Command swallow(BooleanSupplier safe) {
        return goTo(() -> GoosePosition.kStow, () -> false, () -> false, safe)
            .alongWith(
                new CommandBuilder()
                    .onInitialize(() -> hasCoral.set(false))
                    .onExecute(() -> beakMotor.setControl(beakVoltageControl.withOutput(GooseSpeed.kSwallow.voltage())))
                    .onEnd(beakMotor::stopMotor)
            )
            .withName("GooseNeck.swallow()");
    }

    private Command goTo(
        Supplier<GoosePosition> position,
        BooleanSupplier allowGoosing,
        BooleanSupplier left,
        BooleanSupplier safe
    ) {
        Mutable<Double> holdPosition = new Mutable<>(-1.0);
        Mutable<Double> lastTarget = new Mutable<>(0.0);
        Timer timer = new Timer();

        return commandBuilder("GooseNeck.goTo()")
            .onInitialize(() -> {
                timer.stop();
                timer.reset();
                torquePID.reset();
            })
            .onExecute(() -> {
                double target = position.get().rotations() * (left.getAsBoolean() ? -1.0 : 1.0);
                if (lastTarget.value != target) {
                    timer.stop();
                    timer.reset();
                }

                lastTarget.value = target;

                if (allowGoosing.getAsBoolean()) {
                    double currentPosition = getPosition();
                    boolean atPosition = Math2.epsilonEquals(currentPosition, target, kAtPositionTolerance.value());
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
                    timer.stop();
                    timer.reset();
                    torquePID.reset();
                    goosing = false;
                }

                if (!safe.getAsBoolean()) {
                    if (holdPosition.value < 0.0) {
                        double currentPosition = getPosition();
                        GoosePosition close = GoosePosition.closeTo(
                            currentPosition * (left.getAsBoolean() ? -1.0 : 1.0)
                        );
                        holdPosition.value = close != null
                            ? close.rotations() * (left.getAsBoolean() ? -1.0 : 1.0)
                            : currentPosition;
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
