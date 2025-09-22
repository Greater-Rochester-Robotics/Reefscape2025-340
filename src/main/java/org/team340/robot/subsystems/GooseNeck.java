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
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.util.Mutable;
import org.team340.lib.util.command.CommandBuilder;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.RioCAN;
import org.team340.robot.util.ReefSelection;

@Logged
public final class GooseNeck extends GRRSubsystem {

    private static final TunableTable tunables = Tunables.getNested("gooseNeck");

    private static final TunableDouble closeToTolerance = tunables.value("closeToTolerance", 0.08);
    private static final TunableDouble atPositionTolerance = tunables.value("atPositionTolerance", 0.01);
    private static final TunableDouble torqueDelay = tunables.value("torqueDelay", 0.2);
    private static final TunableDouble torqueMax = tunables.value("torqueMax", 12.0);

    /**
     * A position for the goose neck. All positions in this enum should be
     * a positive value, moving the goose neck to the *right* of the robot
     * (CCW+). Moving the goose neck to the left should be specified in
     * method arguments, with the applied rotation being inverted from
     * the values specified here.
     */
    private static enum GoosePosition {
        STOW(0.0),
        SCORE_L1(0.4),
        SCORE(0.5);

        private TunableDouble rotations;

        private GoosePosition(double rotations) {
            this.rotations = tunables.value("positions/" + name(), rotations);
        }

        private double rotations() {
            return rotations.get();
        }

        public static GoosePosition closeTo(double position) {
            for (GoosePosition option : values()) {
                if (Math.abs(option.rotations() - position) < closeToTolerance.get()) {
                    return option;
                }
            }

            return null;
        }
    }

    private static enum GooseSpeed {
        INTAKE(-6.5),
        SEAT(-5.0),
        POOP_L1(-4.25),
        L1(4.0),
        L2_L3(12.0),
        L4(9.25),
        BARF(-8.0),
        SWALLOW(8.0);

        private TunableDouble voltage;

        private GooseSpeed(double voltage) {
            this.voltage = tunables.value("speeds/" + name(), voltage);
        }

        private double voltage() {
            return voltage.get();
        }
    }

    private final TalonFX pivotMotor;
    private final TalonFXS beakMotor;
    private final CANdi candi;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Boolean> beamBreak;

    private final MotionMagicVoltage positionControl;
    private final TorqueCurrentFOC torqueControl;
    private final VoltageOut beakVoltageControl;
    private final VoltageOut beakSensorVoltageControl;

    private final PIDController torquePID = new PIDController(185.0, 0.0, 18.0);
    private final AtomicBoolean hasCoral = new AtomicBoolean(false);
    private boolean goosing = false;

    public GooseNeck() {
        pivotMotor = new TalonFX(RioCAN.GOOSE_NECK);
        beakMotor = new TalonFXS(RioCAN.GOOSE_BEAK);
        candi = new CANdi(RioCAN.GOOSE_CANDI);

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

        pivotConfig.CurrentLimits.StatorCurrentLimit = 50.0;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = 30.0;

        pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        pivotConfig.Feedback.SensorToMechanismRatio = (50.0 / 10.0) * (72.0 / 14.0);

        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 40.0;
        pivotConfig.MotionMagic.MotionMagicAcceleration = 17.5;

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

        beakConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        beakConfig.CurrentLimits.SupplyCurrentLimit = 20.0;

        beakConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        beakConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        beakConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = RioCAN.GOOSE_CANDI;
        beakConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANdiS1;
        beakConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        beakConfig.HardwareLimitSwitch.ReverseLimitEnable = true;

        CANdiConfiguration candiConfig = new CANdiConfiguration();
        candiConfig.PWM2.AbsoluteSensorOffset = 0.927;
        candiConfig.PWM2.SensorDirection = true;

        PhoenixUtil.run(() -> pivotMotor.clearStickyFaults());
        PhoenixUtil.run(() -> beakMotor.clearStickyFaults());
        PhoenixUtil.run(() -> candi.clearStickyFaults());

        PhoenixUtil.run(() -> pivotMotor.getConfigurator().apply(pivotConfig));
        PhoenixUtil.run(() -> beakMotor.getConfigurator().apply(beakConfig));
        PhoenixUtil.run(() -> candi.getConfigurator().apply(candiConfig));

        position = pivotMotor.getPosition();
        velocity = pivotMotor.getVelocity();
        beamBreak = candi.getS1Closed();

        PhoenixUtil.run(() ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                position,
                velocity,
                candi.getPWM2Position(),
                candi.getPWM2Velocity()
            )
        );
        PhoenixUtil.run(() -> beamBreak.setUpdateFrequency(300));
        PhoenixUtil.run(() -> ParentDevice.optimizeBusUtilizationForAll(10, pivotMotor, beakMotor, candi));

        positionControl = new MotionMagicVoltage(0.0);
        torqueControl = new TorqueCurrentFOC(0.0);
        beakVoltageControl = new VoltageOut(0.0);
        beakSensorVoltageControl = new VoltageOut(0.0);

        beakVoltageControl.UpdateFreqHz = 0.0;
        beakVoltageControl.IgnoreHardwareLimits = true;

        beakSensorVoltageControl.UpdateFreqHz = 0.0;

        PhoenixUtil.run(() ->
            pivotMotor.setPosition(
                MathUtil.inputModulus(candi.getPWM2Position(false).waitForUpdate(1.0).getValueAsDouble(), -0.5, 0.5)
            )
        );

        tunables.add("pivotMotor", pivotMotor);
        tunables.add("torquePID", torquePID);

        // Warm-up enums
        GoosePosition.STOW.rotations();
        GooseSpeed.INTAKE.voltage();
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(position, velocity, beamBreak);
    }

    // *************** Helper Functions ***************

    @NotLogged
    public boolean hasCoral() {
        return hasCoral.get() || RobotBase.isSimulation();
    }

    @NotLogged
    public boolean noCoral() {
        return !hasCoral();
    }

    @NotLogged
    public boolean goosing() {
        return goosing;
    }

    @NotLogged
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
        return Commands.runOnce(() -> this.hasCoral.set(hasCoral))
            .ignoringDisable(true)
            .withName("Intake.setHasCoral(" + hasCoral + ")");
    }

    public Command stow(BooleanSupplier safe) {
        return goTo(() -> GoosePosition.STOW, () -> false, () -> false, safe).withName("GooseNeck.stow()");
    }

    public Command intake(
        BooleanSupplier button,
        BooleanSupplier swallow,
        BooleanSupplier inHopper,
        BooleanSupplier safe
    ) {
        enum State {
            INIT,
            SEAT,
            DONE
        }

        Mutable<State> state = new Mutable<>(State.INIT);
        Debouncer debounce = new Debouncer(0.065, DebounceType.kRising);
        Timer delay = new Timer();

        return goTo(() -> GoosePosition.STOW, () -> false, () -> false, safe)
            .withDeadline(
                new CommandBuilder()
                    .onInitialize(() -> {
                        state.value = State.INIT;
                        debounce.calculate(false);
                    })
                    .onExecute(() -> {
                        boolean beamBroken = debounce.calculate(beamBroken());

                        if (swallow.getAsBoolean()) {
                            beakMotor.setControl(beakVoltageControl.withOutput(GooseSpeed.SWALLOW.voltage()));
                            hasCoral.set(false);
                            state.value = State.INIT;
                            return;
                        }

                        switch (state.value) {
                            case INIT:
                                if (!beamBroken || inHopper.getAsBoolean()) {
                                    beakMotor.setControl(beakVoltageControl.withOutput(GooseSpeed.INTAKE.voltage()));
                                    break;
                                }

                                delay.restart();

                                // Fall-through
                                state.value = State.SEAT;
                            case SEAT:
                                if (beamBroken) {
                                    beakMotor.setControl(
                                        beakSensorVoltageControl.withOutput(GooseSpeed.SEAT.voltage())
                                    );
                                    break;
                                }

                                // Fall-through
                                state.value = State.DONE;
                            case DONE:
                                hasCoral.set(true);
                                beakMotor.stopMotor();
                                break;
                        }
                    })
                    .isFinished(() -> hasCoral() || (!button.getAsBoolean() && state.value.equals(State.INIT)))
                    .onEnd(beakMotor::stopMotor)
            )
            .onlyIf(this::noCoral)
            .withName("GooseNeck.intake()");
    }

    public Command score(
        ReefSelection selection,
        BooleanSupplier runManual,
        BooleanSupplier allowGoosing,
        BooleanSupplier safe
    ) {
        Mutable<Boolean> beamTrigger = new Mutable<>(false);

        return goTo(
            () -> !selection.isL1() ? GoosePosition.SCORE : GoosePosition.SCORE_L1,
            () -> !selection.isL1() && allowGoosing.getAsBoolean(),
            selection::isLeft,
            safe
        )
            .alongWith(
                new CommandBuilder()
                    .onInitialize(() -> beamTrigger.value = false)
                    .onExecute(() -> {
                        if (beamBroken() || beamTrigger.value || runManual.getAsBoolean()) {
                            beakMotor.setControl(
                                beakVoltageControl.withOutput(
                                    selection.isL1()
                                        ? GooseSpeed.L1.voltage()
                                        : selection.isL4() ? GooseSpeed.L4.voltage() : GooseSpeed.L2_L3.voltage()
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

    public Command poopL1(BooleanSupplier safe) {
        return goTo(() -> GoosePosition.STOW, () -> false, () -> false, safe)
            .alongWith(
                new CommandBuilder()
                    .onInitialize(() -> hasCoral.set(false))
                    .onExecute(() -> beakMotor.setControl(beakVoltageControl.withOutput(GooseSpeed.POOP_L1.voltage())))
                    .onEnd(beakMotor::stopMotor)
            )
            .withName("GooseNeck.poopL1()");
    }

    public Command barf(BooleanSupplier safe) {
        return goTo(() -> GoosePosition.STOW, () -> false, () -> false, safe)
            .alongWith(
                new CommandBuilder()
                    .onInitialize(() -> hasCoral.set(false))
                    .onExecute(() -> beakMotor.setControl(beakVoltageControl.withOutput(GooseSpeed.BARF.voltage())))
                    .onEnd(beakMotor::stopMotor)
            )
            .withName("GooseNeck.barf()");
    }

    public Command swallow(BooleanSupplier safe) {
        return goTo(() -> GoosePosition.STOW, () -> false, () -> false, safe)
            .alongWith(
                new CommandBuilder()
                    .onInitialize(() -> hasCoral.set(false))
                    .onExecute(() -> beakMotor.setControl(beakVoltageControl.withOutput(GooseSpeed.SWALLOW.voltage())))
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

                double currentPosition = getPosition();
                lastTarget.value = target;

                if (allowGoosing.getAsBoolean()) {
                    boolean atPosition = Math.abs(currentPosition - target) < atPositionTolerance.get();
                    goosing = true;

                    if (timer.hasElapsed(torqueDelay.get())) {
                        if (atPosition) {
                            pivotMotor.stopMotor();
                        } else {
                            pivotMotor.setControl(
                                torqueControl.withOutput(
                                    MathUtil.clamp(
                                        torquePID.calculate(currentPosition, target),
                                        -torqueMax.get(),
                                        torqueMax.get()
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
