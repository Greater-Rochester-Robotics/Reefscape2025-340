package org.team340.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import edu.wpi.first.epilogue.Logged;
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
    private final MotionMagicVoltage positionControl;

    public Elevator() {
        // MOTOR SETUP
        leadMotor = new TalonFX(RobotMap.kElevatorLead, RobotMap.kLowerCANBus);
        followMotor = new TalonFX(RobotMap.kElevatorFollow, RobotMap.kLowerCANBus);

        TalonFXConfiguration motorCfg = new TalonFXConfiguration();

        motorCfg.CurrentLimits.StatorCurrentLimit = 80;
        motorCfg.CurrentLimits.SupplyCurrentLimit = 100;

        motorCfg.Slot0.kP = 0;
        motorCfg.Slot0.kI = 0;
        motorCfg.Slot0.kD = 0;
        motorCfg.Slot0.kG = 0;
        motorCfg.Slot0.kS = 0;
        motorCfg.Slot0.kV = 0;

        motorCfg.MotionMagic.MotionMagicCruiseVelocity = 0;
        motorCfg.MotionMagic.MotionMagicAcceleration = 0;

        motorCfg.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANdiS1;
        motorCfg.HardwareLimitSwitch.ReverseLimitRemoteSensorID = 22;
        motorCfg.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        motorCfg.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        motorCfg.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;

        PhoenixUtil.run("Clear Lead Motor Sticky Faults", leadMotor, () -> leadMotor.clearStickyFaults());
        PhoenixUtil.run("Clear Follow Motor sticky Faults", followMotor, () -> followMotor.clearStickyFaults());
        PhoenixUtil.run("Apply Lead TalonFXConfiguration", leadMotor, () -> leadMotor.getConfigurator().apply(motorCfg)
        );
        PhoenixUtil.run("Apply Follow TalonFXConfiguration", followMotor, () ->
            followMotor.getConfigurator().apply(motorCfg)
        );

        positionControl = new MotionMagicVoltage(0).withSlot(0);
        followMotor.setControl(new Follower(leadMotor.getDeviceID(), false));
    }

    // *************** Helper Functions ***************

    public boolean isAtPosition(ElevatorPosition position) {
        double averagePosition =
            (leadMotor.getPosition().getValueAsDouble() + followMotor.getPosition().getValueAsDouble()) / 2.0;
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
            .isFinished(() -> isAtPosition(position))
            .onEnd(() -> {
                leadMotor.stopMotor();
            });
    }
}
