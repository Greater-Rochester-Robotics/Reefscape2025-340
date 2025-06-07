package org.team340.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
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
        DOWN(0.18),
        INTAKE(0.18),
        BARF(0.18),
        SWALLOW(0.65),
        L1(4.0, Type.SCORING),
        L2(10.75, Type.SCORING),
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
            this.rotations = Tunable.value("elevator/positions/" + name(), rotations);
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
                if (distance < CLOSEST_TOLERANCE.get() && distance <= min) {
                    closest = option;
                    min = distance;
                }
            }

            return closest;
        }
    }

    private static final TunableDouble CLOSEST_TOLERANCE = Tunable.value("elevator/closestTolerance", 0.35);
    private static final TunableDouble AT_POSITION_TOLERANCE = Tunable.value("elevator/atPositionTolerance", 0.5);
    private static final TunableDouble HOMING_VOLTS = Tunable.value("elevator/homingVoltage", -1.0);

    private final TalonFX leadMotor;
    private final TalonFX followMotor;
    private final CANdi candi;

    private final StatusSignal<Angle> leadPosition;
    private final StatusSignal<Angle> followPosition;
    private final StatusSignal<AngularVelocity> leadVelocity;
    private final StatusSignal<AngularVelocity> followVelocity;
    private final StatusSignal<Boolean> limitSwitch;

    private final MotionMagicExpoVoltage positionControl;
    private final VoltageOut voltageControl;
    private final Follower followControl;

    private boolean atPosition = false;
    private boolean scoring = false;
    private boolean dunking = false;
    private boolean homed = false;

    public Elevator() {
        // MOTOR SETUP
        leadMotor = new TalonFX(LowerCAN.ELEVATOR_LEAD, LowerCAN.LOWER_CAN);
        followMotor = new TalonFX(LowerCAN.ELEVATOR_FOLLOW, LowerCAN.LOWER_CAN);
        candi = new CANdi(LowerCAN.ELEVATOR_CANDI, LowerCAN.LOWER_CAN);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.CurrentLimits.StatorCurrentLimit = 100.0;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 80.0;

        motorConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = LowerCAN.ELEVATOR_CANDI;
        motorConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANdiS1;
        motorConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        motorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        motorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0.0;

        motorConfig.MotionMagic.MotionMagicExpo_kV = 0.06;
        motorConfig.MotionMagic.MotionMagicExpo_kA = 0.03;

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motorConfig.Slot0.kP = 4.0;
        motorConfig.Slot0.kI = 0.0;
        motorConfig.Slot0.kD = 0.0;
        motorConfig.Slot0.kG = 0.4;
        motorConfig.Slot0.kS = 0.0;
        motorConfig.Slot0.kV = 0.15;
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
                1000,
                leadMotor.getDutyCycle(),
                leadMotor.getMotorVoltage(),
                leadMotor.getTorqueCurrent()
            )
        );
        PhoenixUtil.run("Optimize Elevator CAN Utilization", () ->
            ParentDevice.optimizeBusUtilizationForAll(20, leadMotor, followMotor, candi)
        );

        positionControl = new MotionMagicExpoVoltage(0.0);
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

    public boolean dunked() {
        var closest = ElevatorPosition.closest(getPosition());
        return dunking && closest != null && closest.type.equals(ElevatorPosition.Type.DUNK);
    }

    public boolean safeForIntake() {
        return getPosition() <= ElevatorPosition.INTAKE.rotations() + CLOSEST_TOLERANCE.get();
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
                    leadMotor.setControl(voltageControl.withOutput(HOMING_VOLTS.get()));
                    homed = limitSwitch.getValue();
                    return;
                }

                ElevatorPosition targetPos = position.get();
                double target = targetPos.rotations();
                double currentPosition = getPosition();
                atPosition = Math.abs(currentPosition - target) < AT_POSITION_TOLERANCE.get();
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

                leadMotor.setControl(positionControl.withPosition(target));
            })
            .onEnd(() -> {
                atPosition = false;
                scoring = false;
                dunking = false;
            });
    }
}
