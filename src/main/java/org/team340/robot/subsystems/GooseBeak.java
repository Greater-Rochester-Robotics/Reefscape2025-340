package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.RevUtil;
import org.team340.robot.Constants;
import org.team340.robot.Constants.RobotMap;

/**
 * Controls the rollers and sensors used to score the coral, not including the pivot.
 */
@Logged
public class GooseBeak extends GRRSubsystem {

    public static enum Speed {
        kIntake(-0.4),
        kScore(1.0),
        kIndexing(0.0);

        private final TunableDouble kPercentOutput;

        private Speed(final double percentOutput) {
            kPercentOutput = Tunable.doubleValue(getEnumName(this), percentOutput);
        }

        public double getPercentOutput() {
            return kPercentOutput.value();
        }
    }

    // private final TalonFXS motor;
    private final SparkMax motor;

    public GooseBeak() {
        // motor = new TalonFXS(RobotMap.kGooseBeakMotor);

        // TalonFXSConfiguration config = new TalonFXSConfiguration();

        // config.CurrentLimits.StatorCurrentLimit = 30.0;
        // config.CurrentLimits.SupplyCurrentLimit = 20.0;

        // config.HardwareLimitSwitch.ReverseLimitSource = RobotMap.kGooseBeamBreak;
        // config.HardwareLimitSwitch.ReverseLimitRemoteSensorID = RobotMap.kGooseCANdi;
        // config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen; // TODO check this
        // config.HardwareLimitSwitch.ReverseLimitEnable = false; // TODO this may change depending on sensor mounting

        // PhoenixUtil.run("Clear Goose Beak Sticky Faults", motor, () -> motor.clearStickyFaults());
        // PhoenixUtil.run("Apply Goose Beak TalonFXSConfiguration", motor, () -> motor.getConfigurator().apply(config));

        motor = new SparkMax(RobotMap.kGooseBeakMotor, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();

        config.smartCurrentLimit(30).idleMode(IdleMode.kCoast).inverted(true);

        RevUtil.config(motor, config);
    }

    // *************** Helper Functions ***************

    /**
     * Stops the roller motor.
     */
    private void stop() {
        motor.stopMotor();
    }

    /**
     * Sets the target speed of the rollers.
     * @param speed The target speed. Speeds should be between 1.0 and -1.0.
     */
    private void setTargetSpeed(double percentOutput) {
        motor.setVoltage(percentOutput * Constants.kVoltage);
    }

    /**
     * Checks if the beam break detects an object.
     * @return True if the beam break detects an object, false otherwise.
     */
    public boolean hasPiece() {
        // TODO check this
        // return motor.getReverseLimit().getValue().equals(ReverseLimitValue.ClosedToGround);
        return false;
    }

    // *************** Commands ***************

    /**
     * Runs the rollers at the speed supplied by {@code speedSupplier}.
     * @param speedSupplier Supplies the speed the rollers are run at. Speeds should be between 1.0 and -1.0
     */
    private Command runAtSpeed(DoubleSupplier speedSupplier) {
        return commandBuilder(getMethodInfo("supplier")) // TODO we should figure out how to log objects
            .onExecute(() -> setTargetSpeed(speedSupplier.getAsDouble()))
            .onEnd(this::stop);
    }

    /**
     * Runs the intake at the {@link GooseneckRollers#kIntakeSpeed kIntakeSpeed}.
     */
    public Command intake() {
        return runAtSpeed(Speed.kIntake::getPercentOutput).withName(getMethodInfo());
    }

    /**
     * Runs the intake at the {@link GooseneckRollers#kScoreSpeed}.
     */
    public Command score() {
        return runAtSpeed(Speed.kScore::getPercentOutput).withName(getMethodInfo());
    }

    /**
     * Indexes coral in the rollers.
     */
    public Command indexPiece() {
        return sequence(
            deadline(sequence(waitUntil(this::hasPiece), waitUntil(() -> !hasPiece())), intake()),
            runAtSpeed(Speed.kIndexing::getPercentOutput).until(this::hasPiece)
        ).withName(getMethodInfo());
    }
}
