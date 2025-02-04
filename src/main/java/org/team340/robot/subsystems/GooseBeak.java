package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.RobotMap;

/**
 * Controls the rollers and sensors used to score the coral, not including the pivot.
 */
@Logged
public class GooseBeak extends GRRSubsystem {

    public static enum GooseSpeed {
        kIntake(0.0),
        kScore(0.0),
        kIndexing(0.0);

        // TODO these variable/method names should denote units
        private final TunableDouble speed;

        private GooseSpeed(double speed) {
            this.speed = Tunable.doubleValue(
                getClass().getEnclosingClass().getSimpleName() + "/" + getClass().getSimpleName() + "/" + name(), // TODO is this necessary?
                speed
            );
        }

        // TODO see Elevator for naming convention
        private double getSpeed() {
            return speed.value();
        }
    }

    private final TalonFXS motor;

    public GooseBeak() {
        motor = new TalonFXS(RobotMap.kGooseBeakMotor);

        TalonFXSConfiguration config = new TalonFXSConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentLimit = 20.0;

        config.HardwareLimitSwitch.ReverseLimitSource = RobotMap.kGooseBeamBreak;
        config.HardwareLimitSwitch.ReverseLimitRemoteSensorID = RobotMap.kGooseCANdi;
        config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen; // TODO check this
        config.HardwareLimitSwitch.ReverseLimitEnable = false; // TODO this may change depending on sensor mounting

        PhoenixUtil.run("Clear Goose Beak Sticky Faults", motor, () -> motor.clearStickyFaults());
        PhoenixUtil.run("Apply Goose Beak TalonFXSConfiguration", motor, () -> motor.getConfigurator().apply(config));
    }

    // *************** Helper Functions ***************

    // TODO we probably don't need all of these methods

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
    private void setTargetSpeed(double speed) {
        // TODO Should be voltage control
        motor.set(speed);
    }

    /**
     * Checks if the beam break detects an object.
     * @return True if the beam break detects an object, false otherwise.
     */
    public boolean hasPiece() {
        // TODO check this
        return motor.getReverseLimit().getValue().equals(ReverseLimitValue.ClosedToGround);
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
        return runAtSpeed(GooseSpeed.kIntake::getSpeed).withName(getMethodInfo());
    }

    /**
     * Runs the intake at the {@link GooseneckRollers#kScoreSpeed}.
     */
    public Command score() {
        return runAtSpeed(GooseSpeed.kScore::getSpeed).withName(getMethodInfo());
    }

    /**
     * Indexes coral in the rollers.
     */
    public Command indexPiece() {
        return sequence(
            deadline(sequence(waitUntil(this::hasPiece), waitUntil(() -> !hasPiece())), intake()),
            runAtSpeed(GooseSpeed.kIndexing::getSpeed).until(this::hasPiece)
        ).withName(getMethodInfo());
    }
}
