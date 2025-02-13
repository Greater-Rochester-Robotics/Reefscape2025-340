package org.team340.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.util.Mutable;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
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
    public static enum Position {
        kIn(0.0),
        kScoreL1(0.7),
        kScoreForward(1.0);

        private TunableDouble kRotations;

        private Position(double rotations) {
            kRotations = Tunable.doubleValue(getEnumName(this), rotations);
        }

        private double getRotations() {
            return kRotations.value();
        }
    }

    private static final TunableDouble kAtPositionTolerance = Tunable.doubleValue("kAtPositionTolerance", 0.05);
    private static final TunableDouble kTorqueTolerance = Tunable.doubleValue("kTorqueTolerance", 0.1);

    private final TalonFX motor;
    private final CANdi candi;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;

    private final MotionMagicVoltage positionControl;
    private final TorqueCurrentFOC torqueControl;

    public GooseNeck() {
        motor = new TalonFX(UpperCAN.kGooseNeckMotor);
        candi = new CANdi(UpperCAN.kGooseCANdi);

        PhoenixUtil.run("Clear Goose Neck Motor Sticky Faults", () -> motor.clearStickyFaults());
        PhoenixUtil.run("Clear Goose Neck CANdi Sticky Faults", () -> candi.clearStickyFaults());

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.CurrentLimits.StatorCurrentLimit = 60.0;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;

        motorConfig.Feedback.FeedbackSensorSource = UpperCAN.kGooseEncoder;
        motorConfig.Feedback.FeedbackRemoteSensorID = UpperCAN.kGooseCANdi;
        motorConfig.Feedback.RotorToSensorRatio = (50.0 / 10.0) * (72.0 / 14.0);

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = 0.0;
        motorConfig.MotionMagic.MotionMagicAcceleration = 0.0;

        motorConfig.Slot0.kP = 1.0;
        motorConfig.Slot0.kI = 0.0;
        motorConfig.Slot0.kD = 0.0;
        motorConfig.Slot0.kS = 0.0;
        motorConfig.Slot0.kV = 0.0;

        motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = motorConfig.CurrentLimits.StatorCurrentLimit;
        motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -motorConfig.CurrentLimits.StatorCurrentLimit;

        CANdiConfiguration candiConfig = new CANdiConfiguration();
        candiConfig.PWM2.AbsoluteSensorOffset = 0.0;
        candiConfig.PWM2.SensorDirection = true;

        PhoenixUtil.run("Apply Goose Neck TalonFXConfiguration", () -> motor.getConfigurator().apply(motorConfig));
        PhoenixUtil.run("Apply Goose Neck CANdiConfiguration", () -> candi.getConfigurator().apply(candiConfig));

        position = motor.getPosition();
        velocity = motor.getVelocity();

        PhoenixUtil.run("Set Goose Neck Position/Velocity Signal Frequencies", () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                position,
                velocity,
                candi.getPWM2Position(),
                candi.getPWM2Velocity()
            )
        );
        PhoenixUtil.run("Optimize Goose Neck CAN Utilization", () ->
            ParentDevice.optimizeBusUtilizationForAll(5, motor, candi)
        );

        positionControl = new MotionMagicVoltage(0.0);
        torqueControl = new TorqueCurrentFOC(0.0);

        Tunable.pidController("GooseNeck/pid", motor);
        Tunable.motionProfile("GooseNeck/motion", motor);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(position, velocity);
    }

    // *************** Helper Functions ***************

    private double getPosition() {
        return BaseStatusSignal.getLatencyCompensatedValueAsDouble(position, velocity);
    }

    // *************** Commands ***************

    /**
     * Moves the pivot to predetermined positions.
     * @param position The position to move the pivot to.
     */
    public Command goTo(Position position, boolean left) {
        Mutable<Boolean> reachedPosition = new Mutable<>(false);

        return commandBuilder(position.name())
            .onInitialize(() -> reachedPosition.accept(false))
            .onExecute(() ->
                motor.setControl(positionControl.withPosition(position.getRotations() * (left ? -1.0 : 1.0)))
            )
            .onEnd(() -> motor.stopMotor());
    }
}
