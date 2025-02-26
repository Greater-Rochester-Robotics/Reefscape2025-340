package org.team340.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
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
        kIntake(0.7),
        kBarf(0.7),
        kSwallow(1.2),
        kBabyBird(12.7),
        kL1(0.0),
        kL2(12.5),
        kL3(26.0),
        kL4(46.1);

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

    private static final TunableDouble kDunkRotations = Tunable.doubleValue("elevator/kDunkRotations", -2.0);
    private static final TunableDouble kCloseToTolerance = Tunable.doubleValue("elevator/kCloseToTolerance", 0.5);
    private static final TunableDouble kHomingVoltage = Tunable.doubleValue("elevator/kHomingVoltage", -1.0);

    private static final TunableDouble kUpVel = Tunable.doubleValue("elevator/motion/kUpVel", 200.0);
    private static final TunableDouble kUpAccel = Tunable.doubleValue("elevator/motion/kUpAccel", 325.0);
    private static final TunableDouble kDownVel = Tunable.doubleValue("elevator/motion/kDownVel", 250.0);
    private static final TunableDouble kDownAccel = Tunable.doubleValue("elevator/motion/kDownAccel", 350.0);

    private final TalonFX leadMotor;
    private final TalonFX followMotor;
    private final CANdi candi;

    private final StatusSignal<Angle> leadPosition;
    private final StatusSignal<Angle> followPosition;
    private final StatusSignal<AngularVelocity> leadVelocity;
    private final StatusSignal<AngularVelocity> followVelocity;
    private final StatusSignal<Boolean> limitSwitch;

    private final DynamicMotionMagicVoltage positionControl;
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

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motorConfig.Slot0.kP = 2.4;
        motorConfig.Slot0.kI = 0.0;
        motorConfig.Slot0.kD = 0.0;
        motorConfig.Slot0.kG = 0.4;
        motorConfig.Slot0.kS = 0.0;
        motorConfig.Slot0.kV = 0.12;

        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 48.7;
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

        positionControl = new DynamicMotionMagicVoltage(0.0, 0.0, 0.0, 0.0);
        voltageControl = new VoltageOut(0.0);
        followControl = new Follower(leadMotor.getDeviceID(), false);

        PhoenixUtil.run("Set Elevator Follow Motor Control", () -> followMotor.setControl(followControl));

        Tunable.pidController("elevator/pid", leadMotor);
        Tunable.pidController("elevator/pid", followMotor);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(leadPosition, followPosition, leadVelocity, followVelocity, limitSwitch);
    }

    // *************** Helper Functions ***************

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

                double currentPosition = getPosition();
                if (!safe.getAsBoolean()) {
                    if (holdPosition.value < 0.0) {
                        ElevatorPosition close = ElevatorPosition.closeTo(currentPosition);
                        holdPosition.value = close != null ? close.rotations() : currentPosition;
                    }

                    leadMotor.setControl(
                        positionControl
                            .withPosition(holdPosition.value + fudge.getAsDouble())
                            .withVelocity(kUpVel.value())
                            .withAcceleration(kUpAccel.value())
                    );
                } else {
                    double target = position.get().rotations() + fudge.getAsDouble();
                    if (target >= currentPosition) {
                        leadMotor.setControl(
                            positionControl
                                .withPosition(target)
                                .withVelocity(kUpVel.value())
                                .withAcceleration(kUpAccel.value())
                        );
                    } else {
                        leadMotor.setControl(
                            positionControl
                                .withPosition(target)
                                .withVelocity(kDownVel.value())
                                .withAcceleration(kDownAccel.value())
                        );
                    }
                    holdPosition.value = -1.0;
                }
            })
            .onEnd(leadMotor::stopMotor);
    }
}
