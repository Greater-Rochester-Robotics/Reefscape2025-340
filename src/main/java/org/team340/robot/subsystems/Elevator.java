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
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
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
        DOWN(0.0),
        INTAKE(0.18),
        BARF(0.18),
        SWALLOW(0.65),
        L1(4.0, true),
        L2(10.75, true),
        L3(22.5, true),
        L4(40.25, true);

        private final TunableDouble rotations;
        private final boolean scoring;

        private ElevatorPosition(double rotations) {
            this(rotations, false);
        }

        private ElevatorPosition(double rotations, boolean scoring) {
            this.rotations = Tunable.doubleValue("elevator/positions/" + name(), rotations);
            this.scoring = scoring;
        }

        public double rotations() {
            return rotations.value();
        }

        private static ElevatorPosition closest(double position) {
            ElevatorPosition closest = null;
            double min = Double.MAX_VALUE;
            for (ElevatorPosition option : values()) {
                double distance = Math.abs(option.rotations() - position);
                if (distance < CLOSEST_TOLERANCE.value() && distance <= min) {
                    closest = option;
                    min = distance;
                }
            }

            return closest;
        }
    }

    private static final TunableDouble DUNK_ROTATIONS = Tunable.doubleValue("elevator/dunkRotations", -3.0);
    private static final TunableDouble CLOSEST_TOLERANCE = Tunable.doubleValue("elevator/closestTolerance", 0.35);
    private static final TunableDouble AT_POSITION_TOLERANCE = Tunable.doubleValue("elevator/atPositionTolerance", 0.2);
    private static final TunableDouble ZERO_TOLERANCE = Tunable.doubleValue("elevator/zeroTolerance", 0.15);
    private static final TunableDouble HOMING_VOLTS = Tunable.doubleValue("elevator/homingVoltage", -1.0);

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

    private boolean atPosition = false;
    private boolean scoring = false;
    private boolean homed = false;

    public Elevator() {
        // MOTOR SETUP
        leadMotor = new TalonFX(LowerCAN.ELEVATOR_LEAD, LowerCAN.LOWER_CAN);
        followMotor = new TalonFX(LowerCAN.ELEVATOR_FOLLOW, LowerCAN.LOWER_CAN);
        candi = new CANdi(LowerCAN.ELEVATOR_CANDI, LowerCAN.LOWER_CAN);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.CurrentLimits.StatorCurrentLimit = 80.0;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 70.0;

        motorConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = LowerCAN.ELEVATOR_CANDI;
        motorConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANdiS1;
        motorConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        motorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        motorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0.0;

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0;
        motorConfig.MotionMagic.MotionMagicAcceleration = 424.0;

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motorConfig.Slot0.kP = 4.0;
        motorConfig.Slot0.kI = 0.0;
        motorConfig.Slot0.kD = 0.0;
        motorConfig.Slot0.kG = 0.58;
        motorConfig.Slot0.kS = 0.0;
        motorConfig.Slot0.kV = 0.16;
        motorConfig.Slot0.kA = 0.004;

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

    @NotLogged
    public boolean atPosition() {
        return atPosition;
    }

    @NotLogged
    public boolean scoring() {
        return scoring;
    }

    public boolean safeForIntake() {
        return getPosition() <= ElevatorPosition.INTAKE.rotations() + CLOSEST_TOLERANCE.value();
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
                        return ElevatorPosition.L1;
                    case 2:
                        return ElevatorPosition.L2;
                    case 3:
                        return ElevatorPosition.L3;
                    case 4:
                        return ElevatorPosition.L4;
                    default:
                        return ElevatorPosition.DOWN;
                }
            },
            () -> {
                if (dunk.getAsBoolean()) dunkinDonuts.value = true;
                if (selection.getLevel() != 4) dunkinDonuts.value = false;
                return dunkinDonuts.value ? DUNK_ROTATIONS.value() : 0.0;
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
                    leadMotor.setControl(voltageControl.withOutput(HOMING_VOLTS.value()));
                    homed = limitSwitch.getValue();
                    return;
                }

                ElevatorPosition targetPos = position.get();
                double target = targetPos.rotations();
                double currentPosition = getPosition();
                atPosition = Math.abs(currentPosition - target) < AT_POSITION_TOLERANCE.value();
                scoring = targetPos.scoring;

                if (!safe.getAsBoolean()) {
                    if (holdPosition.value < 0.0) {
                        ElevatorPosition close = ElevatorPosition.closest(currentPosition);
                        holdPosition.value = close != null ? close.rotations() : currentPosition;
                    }

                    target = holdPosition.value;
                } else {
                    holdPosition.value = -1.0;
                }

                if (currentPosition - ZERO_TOLERANCE.value() <= 0.0 && target - ZERO_TOLERANCE.value() <= 0.0) {
                    leadMotor.stopMotor();
                } else {
                    leadMotor.setControl(positionControl.withPosition(target + fudge.getAsDouble()));
                }
            })
            .onEnd(() -> {
                leadMotor.stopMotor();
                atPosition = false;
                scoring = false;
            });
    }

    public Command thing() {
        return commandBuilder().onInitialize(() -> {}).onExecute(() -> {}).isFinished(false).onEnd(() -> {});
    }
}
