package org.team340.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.RobotMap;

@Logged
public class GooseNeck extends GRRSubsystem {

    public static enum Position {
        kIn(0.0),
        kScoreForward(0.0),
        kScoreLeft(0.0),
        kScoreRight(0.0);

        private TunableDouble kRotations;

        private Position(final double rotations) {
            kRotations = Tunable.doubleValue(getEnumName(this), rotations);
        }

        private double getRotations() {
            return kRotations.value();
        }
    }

    // These are intentionally not tunable, because the upper and lower limits should not be changed
    // while the robot is in normal operation (They should be measured and hardcoded once and then left alone).
    private static final double kUpperLimitRotations = 0.0;
    private static final double kLowerLimitRotations = 0.0;

    private final TalonFX motor;
    private final MotionMagicVoltage controller;

    public GooseNeck() {
        motor = new TalonFX(RobotMap.kGooseNeckMotor);
        controller = new MotionMagicVoltage(0.0);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 60.0;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;

        config.Feedback.FeedbackSensorSource = RobotMap.kGooseEncoder;
        config.Feedback.FeedbackRemoteSensorID = RobotMap.kGooseCANdi;
        config.Feedback.FeedbackRotorOffset = 0.0;
        config.Feedback.RotorToSensorRatio = 1.0; // TODO get this value from mechanical

        config.MotionMagic.MotionMagicCruiseVelocity = 0.0;
        config.MotionMagic.MotionMagicAcceleration = 0.0;

        config.Slot0.kP = 0.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.0;

        PhoenixUtil.run("Clear Goose Neck Sticky Faults", motor, () -> motor.clearStickyFaults());
        PhoenixUtil.run("Apply Goose Neck TalonFXConfiguration", motor, () -> motor.getConfigurator().apply(config));

        Tunable.pidController("GooseNeck/pid", motor);
        Tunable.motionProfile("GooseNeck/motion", motor);
    }

    // *************** Helper Functions ***************

    /**
     * Stops the pivot motor. Should be run at the onEnd of commands.
     */
    private void stop() {
        motor.stopMotor();
    }

    /**
     * Sets the target position of the pivot.
     * @param rotations The position to target in rotations.
     */
    private void setTargetPosition(double rotations) {
        if (rotations > kUpperLimitRotations || rotations < kLowerLimitRotations) {
            DriverStation.reportWarning(
                "The " +
                getName() +
                " position " +
                rotations +
                " rotations must be less than " +
                kUpperLimitRotations +
                " rotations and greater than " +
                kLowerLimitRotations +
                " rotations.",
                false
            );
            return;
        }

        motor.setControl(controller.withPosition(rotations));
    }

    // *************** Commands ***************

    /**
     * Moves the pivot to the position supplied by {@code rotationsSupplier}.
     * @param rotationsSupplier The supplier of the position. Positions should be in rotations.
     */
    private Command goToPosition(DoubleSupplier rotationsSupplier) {
        return commandBuilder(getMethodInfo("supplier"))
            .onExecute(() -> setTargetPosition(rotationsSupplier.getAsDouble()))
            .onEnd(this::stop);
    }

    /**
     * Moves the pivot to predetermined positions.
     * @param position The position to move the pivot to.
     */
    public Command goToPosition(Position position) {
        return goToPosition(position::getRotations).withName(getMethodInfo(position.name()));
    }
}
