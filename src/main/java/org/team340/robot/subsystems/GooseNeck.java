package org.team340.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

    public enum Positions {
        kIn(0.0),
        kScoreForward(0.0),
        kScoreLeft(0.0),
        kScoreRight(0.0);

        private TunableDouble position;

        Positions(double position) {
            this.position = Tunable.doubleValue(
                getClass().getEnclosingClass().getSimpleName() + "/" + getClass().getSimpleName() + "/" + name(), // TODO is this necessary?
                position
            );
        }

        public double getPosition() {
            return position.value();
        }
    }

    // These are intentionally not tunable.
    // TODO why?
    // TODO could be one varibale
    private static final double kUpperLimit = 0.0;
    private static final double kLowerLimit = 0.0;

    private final TalonFX motor;

    public GooseNeck() {
        motor = new TalonFX(RobotMap.kGooseNeckMotor);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimit = 20.0;

        config.Slot0.kP = 0.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.0;

        config.MotionMagic.MotionMagicCruiseVelocity = 0.0;
        config.MotionMagic.MotionMagicAcceleration = 0.0;

        config.Feedback.FeedbackSensorSource = RobotMap.kGooseEncoder;
        config.Feedback.FeedbackRemoteSensorID = RobotMap.kGooseCANdi;
        config.Feedback.FeedbackRotorOffset = 0.0;
        config.Feedback.RotorToSensorRatio = 1.0; // TODO get this value from mechanical

        PhoenixUtil.run("Clear Goose Neck Sticky Faults", motor, () -> motor.clearStickyFaults());
        PhoenixUtil.run("Apply Goose Neck TalonFXConfiguration", motor, () -> motor.getConfigurator().apply(config));
    }

    // *************** Helper Functions ***************

    // TODO we probably don't need all of these methods

    /**
     * Stops the pivot motor. Should be run at the onEnd of commands.
     */
    private void stop() {
        motor.stopMotor();
    }

    /**
     * Sets the target position of the pivot.
     * @param position The position to target in radians.
     */
    private void setTargetPosition(double position) {
        if (position > kUpperLimit || position < kLowerLimit) {
            DriverStation.reportWarning(
                "The " +
                getName() +
                " position " +
                position +
                " must be less than " +
                kUpperLimit +
                " and greater than " +
                kLowerLimit +
                ".",
                false
            );
            return;
        }

        // TODO discuss if radians are needed, we aren't doing any other math with the stored positions
        // TODO that would make radians useful over using the motor's native units.
        final double kInverseTwoPi = 1 / (Math.PI * 2);
        // TODO this sets the position of the motor's encoder, it does not command a new position.
        // TODO use a motion magic control request
        motor.setPosition(position * kInverseTwoPi);
    }

    // *************** Commands ***************

    /**
     * Moves the pivot to the position supplied by {@code positionSupplier}.
     * @param positionSupplier The supplier of the position. Positions should be in radians.
     */
    private Command goToPosition(DoubleSupplier positionSupplier) {
        return commandBuilder(getMethodInfo("supplier"))
            .onExecute(() -> setTargetPosition(positionSupplier.getAsDouble()))
            .onEnd(this::stop);
    }

    /**
     * Moves the pivot to the {@code position}.
     * @param position The position to move the pivot to. The position should be in radians.
     */
    private Command goToPosition(double position) {
        return goToPosition(() -> position).withName(getMethodInfo(String.valueOf(position)));
    }

    /**
     * Moves the pivot to the {@code position}.
     * @param position The position to move the pivot to.
     */
    public Command goToPosition(Positions position) {
        return goToPosition(position::getPosition).withName(getMethodInfo(position.name()));
    }
}
