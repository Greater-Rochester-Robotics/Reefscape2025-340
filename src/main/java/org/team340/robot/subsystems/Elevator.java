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
import java.util.function.Supplier;
import org.team340.lib.util.Math2;
import org.team340.lib.util.Mutable;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.LowerCAN;

@Logged
public class Elevator extends GRRSubsystem {

    public static enum ElevatorPosition {
        kDown(0.0),
        kIntake(0.8),
        kBarf(0.8),
        kSwallow(1.2),
        kL1(0.0),
        kL2(12.5),
        kL3(26.0),
        kL4(46.5);

        private final TunableDouble rotations;

        private ElevatorPosition(double rotations) {
            this.rotations = Tunable.doubleValue("elevator/positions/" + name(), rotations);
        }

        public double rotations() {
            return rotations.value();
        }

        public static ElevatorPosition level(int level) {
            switch (level) {
                case 1:
                    return kL1;
                case 2:
                    return kL2;
                case 3:
                    return kL3;
                case 4:
                    return kL4;
                default:
                    return kDown;
            }
        }

        public static ElevatorPosition closeTo(double position) {
            for (ElevatorPosition option : values()) {
                if (Math2.epsilonEquals(option.rotations(), position, kCloseToTolerance.value())) {
                    return option;
                }
            }

            return null;
        }
    }

    private static final TunableDouble kCloseToTolerance = Tunable.doubleValue("elevator/kCloseToTolerance", 0.75);

    private static final TunableDouble kHomingVoltage = Tunable.doubleValue("elevator/kHomingVoltage", -2.0);
    private static final TunableDouble kTunableVoltage = Tunable.doubleValue("elevator/kTunableVoltage", 0);

    private static final double kMaxPos = 48.7;

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
        motorConfig.CurrentLimits.SupplyCurrentLimit = 100.0;

        motorConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = LowerCAN.kElevatorCANdi;
        motorConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANdiS1;
        motorConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        motorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        motorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0.0;

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = 200.0;
        motorConfig.MotionMagic.MotionMagicAcceleration = 325.0;

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motorConfig.Slot0.kP = 2.0;
        motorConfig.Slot0.kI = 0.0;
        motorConfig.Slot0.kD = 0.0;
        motorConfig.Slot0.kG = 0.4;
        motorConfig.Slot0.kS = 0.0;
        motorConfig.Slot0.kV = 0.1;

        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kMaxPos;
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

        positionControl = new MotionMagicVoltage(0).withSlot(0);
        voltageControl = new VoltageOut(0);
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

    private double getPosition() {
        return (
            (BaseStatusSignal.getLatencyCompensatedValueAsDouble(leadPosition, leadVelocity) +
                BaseStatusSignal.getLatencyCompensatedValueAsDouble(followPosition, followVelocity)) /
            2.0
        );
    }

    // *************** Commands ***************

    public Command goTo(ElevatorPosition position, BooleanSupplier safe) {
        return goTo(() -> position, safe);
    }

    public Command goTo(Supplier<ElevatorPosition> position, BooleanSupplier safe) {
        Mutable<Double> holdPosition = new Mutable<>(-1.0);

        return commandBuilder("Elevator.goTo()")
            .onInitialize(() -> holdPosition.value = -1.0)
            .onExecute(() -> {
                if (!homed) {
                    leadMotor.setControl(voltageControl.withOutput(kHomingVoltage.value()));
                    homed = limitSwitch.getValue();
                }

                if (!safe.getAsBoolean()) {
                    if (holdPosition.value < 0.0) {
                        double currentPosition = getPosition();
                        ElevatorPosition close = ElevatorPosition.closeTo(currentPosition);
                        holdPosition.value = close != null ? close.rotations() : currentPosition;
                    }
                    leadMotor.setControl(positionControl.withPosition(holdPosition.value));
                } else {
                    leadMotor.setControl(positionControl.withPosition(position.get().rotations()));
                    holdPosition.value = -1.0;
                }
            })
            .onEnd(leadMotor::stopMotor);
    }

    public Command applyTunableVoltage() {
        return commandBuilder("Elevator.applyTunableVoltage()")
            .onExecute(() -> leadMotor.setControl(voltageControl.withOutput(kTunableVoltage.value())))
            .onEnd(leadMotor::stopMotor);
    }
}
