package org.team340.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.RevUtil;
import org.team340.robot.Constants;
import org.team340.robot.Constants.UpperCAN;

public class Climber extends GRRSubsystem {

    public static enum ClimberPosition {
        kStore(0.0),
        kDeploy(0.0),
        kClimb(0.0);

        private final TunableDouble angle;

        private ClimberPosition(double angle) {
            this.angle = Tunable.doubleValue("climber/positions/" + name(), angle);
        }

        private double rotations() {
            return angle.value();
        }
    }

    private static final double kForwardLimit = 0.0;
    private static final double kBackwardLimit = 0.0;

    private final SparkFlex motor;
    private final SparkAbsoluteEncoder encoder;

    public Climber() {
        motor = new SparkFlex(UpperCAN.kClimberMotor, MotorType.kBrushless);
        encoder = motor.getAbsoluteEncoder();

        final SparkFlexConfig config = new SparkFlexConfig();

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(-0.5, 0.5);

        config.absoluteEncoder
            .positionConversionFactor(1.0)
            .zeroCentered(true)
            .averageDepth(0)
            .inverted(false)
            .zeroOffset(0.0);

        config.signals.absoluteEncoderPositionAlwaysOn(true).absoluteEncoderPositionPeriodMs(10);

        RevUtil.config(motor, config);
    }

    // *************** Helper Functions ***************

    /**
     * Stops the climber.
     */
    private void stop() {
        motor.stopMotor();
    }

    /**
     * Sets the output voltage of the motor.
     * @param percentOutput The signed percent of the output voltage to set
     */
    private void setTargetSpeed(double percentOutput) {
        motor.setVoltage(percentOutput * Constants.kVoltage);
    }

    /**
     * Gets the number of rotations of the climber's encoder.
     * @return The position in rotations.
     */
    private double getRotations() {
        return encoder.getPosition();
    }

    // *************** Commands ***************

    /**
     * Moves the climber forward until it goes passed the current value of the supplier.
     * @param rotationsSupplier The supplier of the possition that must be surpassed.
     */
    private Command goForwardToPosition(DoubleSupplier rotationsSupplier) {
        return commandBuilder("Climber.goForwardToPosition()")
            .onExecute(() -> setTargetSpeed(1.0))
            .isFinished(() -> getRotations() > rotationsSupplier.getAsDouble() || getRotations() > kForwardLimit)
            .onEnd(this::stop);
    }

    /**
     * Moves the climber backward until it goes passed the current value of the supplier.
     * @param rotationsSupplier The supplier of the possition that must be surpassed.
     */
    private Command goBackwardToPosition(DoubleSupplier rotationsSupplier) {
        return commandBuilder("Climber.goBackwardToPosition()")
            .onExecute(() -> setTargetSpeed(-1.0))
            .isFinished(() -> getRotations() < rotationsSupplier.getAsDouble() || getRotations() < kBackwardLimit)
            .onEnd(this::stop);
    }

    /**
     * Deploys the climber.
     */
    public Command deploy() {
        return goForwardToPosition(ClimberPosition.kDeploy::rotations).withName("Climber.deploy()");
    }

    /**
     * Makes the climber go to the climb position.
     */
    public Command climb() {
        return goForwardToPosition(ClimberPosition.kClimb::rotations).withName("Climber.climb()");
    }
}
