package org.team340.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.DIO;

public class Climber extends GRRSubsystem {

    public static enum Position {
        kStore(0.0),
        kClimb(0.0);

        private final TunableDouble kAngle;

        private Position(final double angle) {
            kAngle = Tunable.doubleValue(getEnumName(this), angle);
        }

        private double getRotations() {
            return kAngle.value();
        }
    }

    private static final double upperLimit = 0.0;
    private static final double lowerLimit = 0.0;

    private final TalonFX climberMotor;
    private final MotionMagicVoltage climberMotorController;
    private final CANcoder climberEncoder;

    public Climber() {
        climberMotor = new TalonFX(DIO.kClimberMotor);
        climberMotorController = new MotionMagicVoltage(0.0);
        climberEncoder = new CANcoder(DIO.kClimberEncoder);

        TalonFXConfiguration motorCfg = new TalonFXConfiguration();

        motorCfg.CurrentLimits.StatorCurrentLimit = 80;
        motorCfg.CurrentLimits.SupplyCurrentLimit = 100;

        motorCfg.Slot0.kP = 0;
        motorCfg.Slot0.kI = 0;
        motorCfg.Slot0.kD = 0;
        motorCfg.Slot0.kG = 0;
        motorCfg.Slot0.kS = 0;
        motorCfg.Slot0.kV = 0;

        motorCfg.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANdiS1;
        motorCfg.HardwareLimitSwitch.ReverseLimitRemoteSensorID = 22;
        motorCfg.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        motorCfg.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        motorCfg.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;

        PhoenixUtil.run("Clear Lead Motor Sticky Faults", climberMotor, () -> climberMotor.clearStickyFaults());

        PhoenixUtil.run("Apply Lead TalonFXConfiguration", climberMotor, () ->
            climberMotor.getConfigurator().apply(motorCfg)
        );
    }

    // *************** Helper Functions ***************

    /**
     * Stops the climber.
     */
    private void stop() {
        climberMotor.stopMotor();
    }

    /**
     * Sets the climber motor to target the specified position.
     * @param position The position to target.
     */
    private void setTargetPosition(double position) {
        if (position > upperLimit || position < lowerLimit) {
            DriverStation.reportError(
                "The inputed position " +
                position +
                " rotations must be between " +
                lowerLimit +
                " rotations and " +
                upperLimit +
                " rotations.",
                null
            );
            return;
        }

        climberMotor.setControl(climberMotorController.withPosition(position));
    }

    // *************** Commands ***************

    /**
     * Makes the climber go to the supplied position.
     * @param rotationsSupplier The supplier of the position in rotations.
     */
    private Command goToPosition(DoubleSupplier rotationsSupplier) {
        return commandBuilder().onExecute(() -> setTargetPosition(rotationsSupplier.getAsDouble())).onEnd(this::stop);
    }

    /**
     * Makes the climber go to the climb position.
     */
    public Command climb() {
        return goToPosition(Position.kClimb::getRotations).withName(getMethodInfo());
    }

    /**
     * Makes the climber go to the safe position.
     */
    public Command store() {
        return goToPosition(Position.kStore::getRotations).withName(getMethodInfo());
    }
}
