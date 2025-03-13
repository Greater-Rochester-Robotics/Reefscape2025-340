package org.team340.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.team340.lib.util.Math2;
import org.team340.lib.util.Mutable;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.LowerCAN;
import org.team340.robot.util.ReefSelection;

@Logged
public final class Elevator extends GRRSubsystem {

    public static enum ElevatorPosition {
        kDown(0.0),
        kIntake(0.5),
        kBarf(0.5),
        kSwallow(0.9),
        kBabyBird(10.9),
        kL1(0.0),
        kL2(10.5),
        kL3(22.5),
        kL4(40.25);

        private final TunableDouble rotations;

        private ElevatorPosition(double rotations) {
            this.rotations = Tunable.doubleValue("elevator/positions/" + name(), rotations);
        }

        public double rotations() {
            return rotations.value();
        }

        private static ElevatorPosition closeTo(double position) {
            ElevatorPosition closest = null;
            double min = Double.MAX_VALUE;
            for (ElevatorPosition option : values()) {
                double distance = Math.abs(option.rotations() - position);
                if (Math2.epsilonEquals(0.0, distance, kCloseToTolerance.value()) && distance <= min) {
                    closest = option;
                    min = distance;
                }
            }

            return closest;
        }
    }

    private static final TunableDouble kDunkRotations = Tunable.doubleValue("elevator/kDunkRotations", -3.0);
    private static final TunableDouble kCloseToTolerance = Tunable.doubleValue("elevator/kCloseToTolerance", 0.35);
    private static final TunableDouble kZeroTolerance = Tunable.doubleValue("elevator/kZeroTolerance", 0.15);
    private static final TunableDouble kHomingVoltage = Tunable.doubleValue("elevator/kHomingVoltage", -1.0);
    private static final TunableDouble kTunableVoltage = Tunable.doubleValue("elevator/kTunableVoltage", 0.0);

    private final TalonFX leadMotor;
    private final TalonFX followMotor;
    private final CANdi candi;

    private final StatusSignal<Angle> leadPosition;
    private final StatusSignal<Angle> followPosition;
    private final StatusSignal<AngularVelocity> leadVelocity;
    private final StatusSignal<AngularVelocity> followVelocity;
    private final StatusSignal<Boolean> limitSwitch;

    private final MotionMagicVoltage positionControl;
    private final VoltageOut voltageControl;
    private final Follower followControl;

    private boolean homed = false;

    public Elevator() {
        // MOTOR SETUP
        leadMotor = new TalonFX(LowerCAN.kElevatorLead, LowerCAN.kLowerCANBus);
        followMotor = new TalonFX(LowerCAN.kElevatorFollow, LowerCAN.kLowerCANBus);
        candi = new CANdi(LowerCAN.kElevatorCANdi, LowerCAN.kLowerCANBus);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.CurrentLimits.StatorCurrentLimit = 80.0;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 70.0;

        motorConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = LowerCAN.kElevatorCANdi;
        motorConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANdiS1;
        motorConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        motorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        motorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0.0;

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0;
        motorConfig.MotionMagic.MotionMagicAcceleration = 424.0;

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motorConfig.Slot0.kP = 1.2;
        motorConfig.Slot0.kI = 0.0;
        motorConfig.Slot0.kD = 0.0;
        motorConfig.Slot0.kG = 0.45;
        motorConfig.Slot0.kS = 0.0;
        motorConfig.Slot0.kV = 0.136;
        motorConfig.Slot0.kA = 0.003;

        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 42.75;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        CANdiConfiguration candiConfig = new CANdiConfiguration();

        PhoenixUtil.run("Clear Elevator Lead Sticky Faults", () -> leadMotor.clearStickyFaults());
        PhoenixUtil.run("Clear Elevator Follow Sticky Faults", () -> followMotor.clearStickyFaults());
        PhoenixUtil.run("Clear Elevator CANdi Sticky Faults", () -> candi.clearStickyFaults());
        PhoenixUtil.run("Apply Elevator Lead TalonFXConfiguration", () -> leadMotor.getConfigurator().apply(motorConfig)
        );
        PhoenixUtil.run("Apply Elevator Follow TalonFXConfiguration", () ->
            followMotor.getConfigurator().apply(motorConfig)
        );
        PhoenixUtil.run("Apply Elevator CANdiConfiguration", () -> candi.getConfigurator().apply(candiConfig));

        leadPosition = leadMotor.getPosition();
        followPosition = followMotor.getPosition();
        leadVelocity = leadMotor.getVelocity();
        followVelocity = followMotor.getVelocity();
        limitSwitch = candi.getS1Closed();

        PhoenixUtil.run("Set Elevator Signal Frequencies", () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                200,
                leadPosition,
                followPosition,
                leadVelocity,
                followVelocity,
                limitSwitch,
                candi.getS1State()
            )
        );
        PhoenixUtil.run("Set Elevator Signal Frequencies for Following", () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                500,
                leadMotor.getDutyCycle(),
                leadMotor.getMotorVoltage(),
                leadMotor.getTorqueCurrent()
            )
        );
        PhoenixUtil.run("Optimize Elevator CAN Utilization", () ->
            ParentDevice.optimizeBusUtilizationForAll(20, leadMotor, followMotor, candi)
        );

        positionControl = new MotionMagicVoltage(0.0);
        voltageControl = new VoltageOut(0.0);
        followControl = new Follower(leadMotor.getDeviceID(), false);

        PhoenixUtil.run("Set Elevator Follow Motor Control", () -> followMotor.setControl(followControl));

        Tunable.pidController("elevator/pid", leadMotor);
        Tunable.pidController("elevator/pid", followMotor);
        Tunable.motionProfile("elevator/motion", leadMotor);
        Tunable.motionProfile("elevator/motion", followMotor);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(leadPosition, followPosition, leadVelocity, followVelocity, limitSwitch);
    }

    // *************** Helper Functions ***************

    public boolean safeForIntake() {
        return getPosition() <= ElevatorPosition.kIntake.rotations() + kCloseToTolerance.value();
    }

    /**
     * Gets the elevator's current position, in rotations.
     */
    private double getPosition() {
        return (
            (BaseStatusSignal.getLatencyCompensatedValueAsDouble(leadPosition, leadVelocity) +
                BaseStatusSignal.getLatencyCompensatedValueAsDouble(followPosition, followVelocity)) /
            2.0
        );
    }

    // *************** Commands ***************

    public Command applyTunableVoltage() {
        return commandBuilder("Elevator.applyTunableVoltage()")
            .onExecute(() -> leadMotor.setControl(voltageControl.withOutput(kTunableVoltage.value())))
            .onEnd(leadMotor::stopMotor);
    }

    /**
     * Goes to a scoring position.
     * @param selection The reef selection helper.
     * @param safe If the elevator is safe to move.
     */
    public Command score(ReefSelection selection, BooleanSupplier dunk, BooleanSupplier safe) {
        Mutable<Boolean> dunkinDonuts = new Mutable<>(false);

        return goTo(
            () -> {
                switch (selection.getLevel()) {
                    case 1:
                        return ElevatorPosition.kL1;
                    case 2:
                        return ElevatorPosition.kL2;
                    case 3:
                        return ElevatorPosition.kL3;
                    case 4:
                        return ElevatorPosition.kL4;
                    default:
                        return ElevatorPosition.kDown;
                }
            },
            () -> {
                if (dunk.getAsBoolean()) dunkinDonuts.value = true;
                if (selection.getLevel() != 4) dunkinDonuts.value = false;
                return dunkinDonuts.value ? kDunkRotations.value() : 0.0;
            },
            safe
        ).beforeStarting(() -> dunkinDonuts.value = false);
    }

    /**
     * Goes to a position.
     * @param position The position to go to.
     * @param safe If the elevator is safe to move.
     */
    public Command goTo(ElevatorPosition position, BooleanSupplier safe) {
        return goTo(() -> position, () -> 0.0, safe);
    }

    /**
     * Goes to a position.
     * @param position The position to go to.
     * @param safe If the elevator is safe to move.
     */
    private Command goTo(Supplier<ElevatorPosition> position, DoubleSupplier fudge, BooleanSupplier safe) {
        Mutable<Double> holdPosition = new Mutable<>(-1.0);

        return commandBuilder("Elevator.goTo()")
            .onInitialize(() -> holdPosition.value = -1.0)
            .onExecute(() -> {
                if (!homed) {
                    leadMotor.setControl(voltageControl.withOutput(kHomingVoltage.value()));
                    homed = limitSwitch.getValue();
                    return;
                }

                double target = position.get().rotations();
                double currentPosition = getPosition();
                if (!safe.getAsBoolean()) {
                    if (holdPosition.value < 0.0) {
                        ElevatorPosition close = ElevatorPosition.closeTo(currentPosition);
                        holdPosition.value = close != null ? close.rotations() : currentPosition;
                    }

                    target = holdPosition.value;
                } else {
                    holdPosition.value = -1.0;
                }

                if (currentPosition - kZeroTolerance.value() <= 0.0 && target - kZeroTolerance.value() <= 0.0) {
                    leadMotor.stopMotor();
                } else {
                    leadMotor.setControl(positionControl.withPosition(target + fudge.getAsDouble()));
                }
            })
            .onEnd(leadMotor::stopMotor);
    }

    public Command thing() {
        return commandBuilder().onInitialize(() -> {}).onExecute(() -> {}).isFinished(false).onEnd(() -> {});
    }
}
