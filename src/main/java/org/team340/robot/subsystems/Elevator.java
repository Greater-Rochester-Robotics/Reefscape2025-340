package org.team340.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.util.Math2;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.RobotMap;

@Logged
public class Elevator extends GRRSubsystem {

    public static enum ElevatorPosition {
        kDown(0),
        kL1(0),
        kL2(0),
        kL3(0),
        kL4(0);

        private final TunableDouble rotations;

        private ElevatorPosition(double rotations) {
            this.rotations = Tunable.doubleValue("Elevator/Positions/" + this.name(), rotations);
        }

        private double rotations() {
            return rotations.value();
        }
    }

    private final TalonFX leadMotor;
    private final TalonFX followMotor;

    private final StatusSignal<Angle> leadPosition;
    private final StatusSignal<Angle> followPosition;
    private final MotionMagicVoltage positionControl;

    public Elevator() {
        // MOTOR SETUP
        leadMotor = new TalonFX(RobotMap.kElevatorLead, RobotMap.kLowerCANBus);
        followMotor = new TalonFX(RobotMap.kElevatorFollow, RobotMap.kLowerCANBus);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 80;
        config.CurrentLimits.SupplyCurrentLimit = 100;

        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kG = 0;
        config.Slot0.kS = 0;
        config.Slot0.kV = 0;

        config.MotionMagic.MotionMagicCruiseVelocity = 0;
        config.MotionMagic.MotionMagicAcceleration = 0;

        config.HardwareLimitSwitch.ReverseLimitSource = RobotMap.kElevatorLimitPort;
        config.HardwareLimitSwitch.ReverseLimitRemoteSensorID = RobotMap.kElevatorCANdi;
        config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;

        PhoenixUtil.run("Clear Elevator Lead Sticky Faults", leadMotor, () -> leadMotor.clearStickyFaults());
        PhoenixUtil.run("Clear Elevator Follow Sticky Faults", followMotor, () -> followMotor.clearStickyFaults());
        PhoenixUtil.run("Apply Elevator Lead TalonFXConfiguration", leadMotor, () ->
            leadMotor.getConfigurator().apply(config)
        );
        PhoenixUtil.run("Apply Elevator Follow TalonFXConfiguration", followMotor, () ->
            followMotor.getConfigurator().apply(config)
        );

        leadPosition = leadMotor.getPosition();
        followPosition = followMotor.getPosition();

        positionControl = new MotionMagicVoltage(0).withSlot(0);
        followMotor.setControl(new Follower(leadMotor.getDeviceID(), false));
    }

    // *************** Helper Functions ***************

    public boolean isAtPosition(ElevatorPosition position) {
        // TODO Latency compensation
        double averagePosition = (leadPosition.getValueAsDouble() + followPosition.getValueAsDouble()) / 2.0;

        // TODO Define epsilon
        return Math2.epsilonEquals(averagePosition, position.rotations());
    }

    // *************** Commands ***************

    /**
     * Move elevator to predetermined positions.
     * @param position - The predetermined positions.
     */
    public Command goTo(ElevatorPosition position) {
        return commandBuilder("Elevator.goTo()")
            .onExecute(() -> {
                // Setting lead motor also sets the follow motor
                leadMotor.setControl(positionControl.withPosition(position.rotations()));
            })
            .isFinished(() -> isAtPosition(position)) // TODO maybe remove for scheduling purposes?
            .onEnd(() -> {
                leadMotor.stopMotor();
            });
    }
}
