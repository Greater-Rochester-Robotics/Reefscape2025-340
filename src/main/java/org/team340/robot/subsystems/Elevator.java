package org.team340.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.util.Math2;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.LowerCAN;

@Logged
public class Elevator extends GRRSubsystem {

    public static enum Position {
        kDown(0.0),
        kL1(10.0),
        kL2(20.0),
        kL3(30.0),
        kL4(40.0);

        private final TunableDouble kRotations;

        private Position(final double rotations) {
            kRotations = Tunable.doubleValue(getEnumName(this), rotations);
        }

        public double getRotations() {
            return kRotations.value();
        }
    }

    private static final TunableDouble kAtPositionEpsilon = Tunable.doubleValue(
        getSubsystemName() + "/kAtPositionEpsilon",
        1e-6
    );
    private static final TunableDouble kTunableVoltage = Tunable.doubleValue(
        getSubsystemName() + "/kTunableVoltage",
        0
    );

    private final TalonFX leadMotor;
    private final TalonFX followMotor;

    private final StatusSignal<Angle> leadPosition;
    private final StatusSignal<Angle> followPosition;
    private final StatusSignal<AngularVelocity> leadVelocity;
    private final StatusSignal<AngularVelocity> followVelocity;

    private final MotionMagicVoltage positionControl;
    private final VoltageOut voltageControl;
    private final Follower followControl;

    public Elevator() {
        // MOTOR SETUP
        leadMotor = new TalonFX(LowerCAN.kElevatorLead, LowerCAN.kLowerCANBus);
        followMotor = new TalonFX(LowerCAN.kElevatorFollow, LowerCAN.kLowerCANBus);

        PhoenixUtil.run("Clear Elevator Lead Sticky Faults", () -> leadMotor.clearStickyFaults());
        PhoenixUtil.run("Clear Elevator Follow Sticky Faults", () -> followMotor.clearStickyFaults());

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.SupplyCurrentLimit = 100.0;

        config.HardwareLimitSwitch.ReverseLimitSource = LowerCAN.kElevatorLimitPort;
        config.HardwareLimitSwitch.ReverseLimitRemoteSensorID = LowerCAN.kElevatorCANdi;
        config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen; // TODO check this
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0.0;

        config.MotionMagic.MotionMagicCruiseVelocity = 120.0;
        config.MotionMagic.MotionMagicAcceleration = 300.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kP = 0.6;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kG = 0.4;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.15;

        PhoenixUtil.run("Apply Elevator Lead TalonFXConfiguration", () -> leadMotor.getConfigurator().apply(config));
        PhoenixUtil.run("Apply Elevator Follow TalonFXConfiguration", () -> followMotor.getConfigurator().apply(config)
        );

        leadPosition = leadMotor.getPosition();
        followPosition = followMotor.getPosition();
        leadVelocity = leadMotor.getVelocity();
        followVelocity = followMotor.getVelocity();

        PhoenixUtil.run("Set Elevator Position/Velocity Signal Frequencies", () ->
            BaseStatusSignal.setUpdateFrequencyForAll(200, leadPosition, followPosition, leadVelocity, followVelocity)
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
            ParentDevice.optimizeBusUtilizationForAll(5, leadMotor, followMotor)
        );

        positionControl = new MotionMagicVoltage(0).withSlot(0);
        voltageControl = new VoltageOut(0);
        followControl = new Follower(leadMotor.getDeviceID(), false);

        PhoenixUtil.run("Set Elevator Follow Motor Control", () -> followMotor.setControl(followControl));

        Tunable.pidController(getName() + "/pid", leadMotor);
        Tunable.pidController(getName() + "/pid", followMotor);
        Tunable.motionProfile(getName() + "/motion", leadMotor);
        Tunable.motionProfile(getName() + "/motion", followMotor);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(leadPosition, followPosition, leadVelocity, followVelocity);
    }

    // *************** Helper Functions ***************

    public boolean isAtPosition(Position position) {
        double averagePosition =
            (BaseStatusSignal.getLatencyCompensatedValueAsDouble(leadPosition, leadVelocity) +
                BaseStatusSignal.getLatencyCompensatedValueAsDouble(followPosition, followVelocity)) /
            2.0;
        return Math2.epsilonEquals(averagePosition, position.getRotations(), kAtPositionEpsilon.value());
    }

    // *************** Commands ***************

    /**
     * Move elevator to predetermined positions.
     * @param position - The predetermined positions.
     */
    public Command goTo(Position position) {
        return commandBuilder(position.name())
            .onExecute(() -> {
                // Setting lead motor also sets the follow motor
                leadMotor.setControl(positionControl.withPosition(position.getRotations()));
            })
            .onEnd(() -> {
                leadMotor.stopMotor();
            });
    }

    public Command applyTunableVoltage() {
        return commandBuilder()
            .onExecute(() -> {
                // Setting lead motor also sets the follow motor
                leadMotor.setControl(voltageControl.withOutput(kTunableVoltage.value()));
            })
            .onEnd(() -> {
                leadMotor.stopMotor();
            });
    }
}
