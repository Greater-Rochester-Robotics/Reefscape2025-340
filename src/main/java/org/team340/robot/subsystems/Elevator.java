package org.team340.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
import java.util.function.Supplier;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.util.Mutable;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.LowerCAN;
import org.team340.robot.util.ReefSelection;

@Logged
public final class Elevator extends GRRSubsystem {

    private static final TunableTable tunables = Tunables.getNested("elevator");

    private static final TunableDouble closestTolerance = tunables.value("closestTolerance", 0.35);
    private static final TunableDouble atPositionTolerance = tunables.value("atPositionTolerance", 2.5);
    private static final TunableDouble emergencyRPS = tunables.value("emergencyRPS", -25.0);
    private static final TunableDouble homingVolts = tunables.value("homingVolts", -1.0);

    public static enum ElevatorPosition {
        DOWN(1.0),
        INTAKE(0.1),
        BARF(0.0),
        SWALLOW(0.65),
        L1(4.0, Type.SCORING),
        L2(10.9, Type.SCORING),
        L3(22.5, Type.SCORING),
        L4(40.25, Type.SCORING),
        L2_DUNK(8.25, Type.DUNK),
        L3_DUNK(18.25, Type.DUNK),
        L4_DUNK(34.0, Type.DUNK);

        private static enum Type {
            SCORING,
            DUNK,
            OTHER
        }

        private final TunableDouble rotations;
        private final Type type;

        private ElevatorPosition(double rotations) {
            this(rotations, Type.OTHER);
        }

        private ElevatorPosition(double rotations, Type type) {
            this.rotations = tunables.value("positions/" + name(), rotations);
            this.type = type;
        }

        public double rotations() {
            return rotations.get();
        }

        private static ElevatorPosition closest(double position) {
            ElevatorPosition closest = null;
            double min = Double.MAX_VALUE;
            for (ElevatorPosition option : values()) {
                double distance = Math.abs(option.rotations() - position);
                if (distance < closestTolerance.get() && distance <= min) {
                    closest = option;
                    min = distance;
                }
            }

            return closest;
        }
    }

    private final TalonFX leadMotor;
    private final TalonFX followMotor;
    private final CANdi candi;

    private final StatusSignal<Angle> leadPosition;
    private final StatusSignal<Angle> followPosition;
    private final StatusSignal<AngularVelocity> leadVelocity;
    private final StatusSignal<AngularVelocity> followVelocity;
    private final StatusSignal<Boolean> limitSwitch;

    private final MotionMagicVoltage positionControl;
    private final VelocityVoltage velocityControl;
    private final VoltageOut voltageControl;
    private final Follower followControl;

    private boolean atPosition = false;
    private boolean scoring = false;
    private boolean dunking = false;
    private boolean homed = false;

    public Elevator() {
        leadMotor = new TalonFX(LowerCAN.ELEVATOR_LEAD, LowerCAN.LOWER_CAN);
        followMotor = new TalonFX(LowerCAN.ELEVATOR_FOLLOW, LowerCAN.LOWER_CAN);
        candi = new CANdi(LowerCAN.ELEVATOR_CANDI, LowerCAN.LOWER_CAN);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.CurrentLimits.StatorCurrentLimit = 120.0;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 80.0;

        motorConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = LowerCAN.ELEVATOR_CANDI;
        motorConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANdiS1;
        motorConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        motorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        motorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0.0;

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = 92.5;
        motorConfig.MotionMagic.MotionMagicAcceleration = 424.0;

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motorConfig.Slot0.kP = 4.0;
        motorConfig.Slot0.kI = 0.0;
        motorConfig.Slot0.kD = 0.0;
        motorConfig.Slot0.kG = 0.57;
        motorConfig.Slot0.kS = 0.0;
        motorConfig.Slot0.kV = 0.158;
        motorConfig.Slot0.kA = 0.004;

        motorConfig.Slot1.kP = 6.0;
        motorConfig.Slot1.kI = 0.0;
        motorConfig.Slot1.kD = 0.06;
        motorConfig.Slot1.kG = 0.57;
        motorConfig.Slot1.kS = 0.0;
        motorConfig.Slot1.kV = 0.1;
        motorConfig.Slot1.kA = 0.004;

        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 40.5;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        CANdiConfiguration candiConfig = new CANdiConfiguration();

        PhoenixUtil.run(() -> leadMotor.clearStickyFaults());
        PhoenixUtil.run(() -> followMotor.clearStickyFaults());
        PhoenixUtil.run(() -> candi.clearStickyFaults());

        PhoenixUtil.run(() -> leadMotor.getConfigurator().apply(motorConfig));
        PhoenixUtil.run(() -> followMotor.getConfigurator().apply(motorConfig));
        PhoenixUtil.run(() -> candi.getConfigurator().apply(candiConfig));

        leadPosition = leadMotor.getPosition();
        followPosition = followMotor.getPosition();
        leadVelocity = leadMotor.getVelocity();
        followVelocity = followMotor.getVelocity();
        limitSwitch = candi.getS1Closed();

        PhoenixUtil.run(() ->
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
        PhoenixUtil.run(() ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                1000,
                leadMotor.getDutyCycle(),
                leadMotor.getMotorVoltage(),
                leadMotor.getTorqueCurrent()
            )
        );
        PhoenixUtil.run(() -> ParentDevice.optimizeBusUtilizationForAll(50, leadMotor, followMotor, candi));

        positionControl = new MotionMagicVoltage(0.0);
        velocityControl = new VelocityVoltage(0.0);
        voltageControl = new VoltageOut(0.0);
        followControl = new Follower(leadMotor.getDeviceID(), false);

        PhoenixUtil.run(() -> followMotor.setControl(followControl));

        tunables.add("motor", leadMotor);
        tunables.add("motor", followMotor);
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

    public boolean dunked() {
        var closest = ElevatorPosition.closest(getPosition());
        return dunking && closest != null && closest.type.equals(ElevatorPosition.Type.DUNK);
    }

    public boolean safeForIntake() {
        return getPosition() <= ElevatorPosition.INTAKE.rotations() + closestTolerance.get();
    }

    public boolean safeForSwallow() {
        return getPosition() <= ElevatorPosition.SWALLOW.rotations() + closestTolerance.get();
    }

    /**
     * Gets the elevator's current position, in rotations.
     */
    private double getPosition() {
        return (
            (BaseStatusSignal.getLatencyCompensatedValueAsDouble(leadPosition, leadVelocity)
                + BaseStatusSignal.getLatencyCompensatedValueAsDouble(followPosition, followVelocity))
            / 2.0
        );
    }

    // *************** Commands ***************

    public Command optimisticIdle(ReefSelection selection, BooleanSupplier hasCoral, BooleanSupplier safe) {
        return goTo(
            () -> !selection.isL1() && hasCoral.getAsBoolean() ? ElevatorPosition.L2 : ElevatorPosition.DOWN,
            safe
        );
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
                if (dunk.getAsBoolean()) dunkinDonuts.value = true;

                switch (selection.getLevel()) {
                    case 1:
                        return ElevatorPosition.L1;
                    case 2:
                        return !dunkinDonuts.value ? ElevatorPosition.L2 : ElevatorPosition.L2_DUNK;
                    case 3:
                        return !dunkinDonuts.value ? ElevatorPosition.L3 : ElevatorPosition.L3_DUNK;
                    case 4:
                        return !dunkinDonuts.value ? ElevatorPosition.L4 : ElevatorPosition.L4_DUNK;
                    default:
                        return ElevatorPosition.DOWN;
                }
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
        return goTo(() -> position, safe);
    }

    public Command emergency() {
        return commandBuilder("Elevator.emergency()")
            .onExecute(() -> leadMotor.setControl(velocityControl.withVelocity(emergencyRPS.get())))
            .onEnd(leadMotor::stopMotor);
    }

    /**
     * Goes to a position.
     * @param position The position to go to.
     * @param safe If the elevator is safe to move.
     */
    private Command goTo(Supplier<ElevatorPosition> position, BooleanSupplier safe) {
        Mutable<Double> holdPosition = new Mutable<>(-1.0);

        return commandBuilder("Elevator.goTo()")
            .onInitialize(() -> holdPosition.value = -1.0)
            .onExecute(() -> {
                if (!homed) {
                    leadMotor.setControl(voltageControl.withOutput(homingVolts.get()));
                    homed = limitSwitch.getValue();
                    return;
                }

                ElevatorPosition targetPos = position.get();
                double target = targetPos.rotations();
                double currentPosition = getPosition();
                atPosition = Math.abs(currentPosition - target) < atPositionTolerance.get();
                scoring = !targetPos.type.equals(ElevatorPosition.Type.OTHER);
                dunking = targetPos.type.equals(ElevatorPosition.Type.DUNK);

                if (!safe.getAsBoolean() && !dunking) {
                    if (holdPosition.value < 0.0) {
                        ElevatorPosition close = ElevatorPosition.closest(currentPosition);
                        holdPosition.value = close != null ? close.rotations() : currentPosition;
                    }

                    target = holdPosition.value;
                } else {
                    holdPosition.value = -1.0;
                }

                int slot = targetPos.type.equals(ElevatorPosition.Type.OTHER) ? 1 : 0;
                leadMotor.setControl(positionControl.withPosition(target).withSlot(slot));
            })
            .onEnd(() -> {
                atPosition = false;
                scoring = false;
                dunking = false;
            });
    }
}
