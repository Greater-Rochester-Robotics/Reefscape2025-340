package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants;
import org.team340.robot.Constants.UpperCAN;

/**
 * Controls the rollers and sensors used to score the coral, not including the pivot.
 */
@Logged
public class GooseBeak extends GRRSubsystem {

    public static enum Speed {
        kIntake(-0.4),
        kScore(1.0);

        private final TunableDouble kPercentOutput;

        private Speed(final double percentOutput) {
            kPercentOutput = Tunable.doubleValue(getEnumName(this), percentOutput);
        }

        public double getPercentOutput() {
            return kPercentOutput.value();
        }
    }

    private static double kIntakeDelaySeconds = 0.5;
    private static double kScoreDelaySeconds = 0.5;

    private final TalonFXS motor;
    private boolean debouncedBeamBroken;
    private boolean hasCoral = true;

    public GooseBeak() {
        motor = new TalonFXS(UpperCAN.kGooseBeakMotor);

        TalonFXSConfiguration config = new TalonFXSConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentLimit = 20.0;

        config.HardwareLimitSwitch.ReverseLimitSource = UpperCAN.kGooseBeamBreak;
        config.HardwareLimitSwitch.ReverseLimitRemoteSensorID = UpperCAN.kGooseCANdi;
        config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        config.HardwareLimitSwitch.ReverseLimitEnable = false;

        PhoenixUtil.run("Clear Goose Beak Sticky Faults", motor, () -> motor.clearStickyFaults());
        PhoenixUtil.run("Apply Goose Beak TalonFXSConfiguration", motor, () -> motor.getConfigurator().apply(config));

        debouncedBeamBroken = beamBroken();
        new Trigger(this::beamBroken)
            .debounce(kIntakeDelaySeconds, DebounceType.kBoth)
            .onTrue(runOnce(() -> debouncedBeamBroken = true))
            .onFalse(runOnce(() -> debouncedBeamBroken = false));
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
    public boolean beamBroken() {
        return motor.getReverseLimit().getValue().equals(ReverseLimitValue.ClosedToGround);
    }

    public boolean hasCoral() {
        return hasCoral;
    }

    public void setHasCoral(boolean hasCoral) {
        this.hasCoral = hasCoral;
    }

    // *************** Commands ***************

    /**
     * Runs the rollers at the speed supplied by {@code speedSupplier}.
     * @param speedSupplier Supplies the speed the rollers are run at. Speeds should be between 1.0 and -1.0
     */
    private Command runAtSpeed(DoubleSupplier speedSupplier) {
        return commandBuilder("supplier") // TODO we should figure out how to log objects
            .onExecute(() -> setTargetSpeed(speedSupplier.getAsDouble()))
            .onEnd(this::stop);
    }

    /**
     * Runs the intake at the {@link GooseneckRollers#kIntakeSpeed kIntakeSpeed}.
     */
    public Command intake() {
        return deadline(
            sequence(
                waitUntil(() -> debouncedBeamBroken),
                waitUntil(() -> !debouncedBeamBroken),
                waitSeconds(kIntakeDelaySeconds),
                runOnce(() -> hasCoral = true)
            ),
            runAtSpeed(Speed.kIntake::getPercentOutput)
        ).withName(getMethodInfo());
    }

    /**
     * Runs the intake at the {@link GooseneckRollers#kScoreSpeed}.
     */
    public Command score() {
        return parallel(
            runAtSpeed(Speed.kScore::getPercentOutput),
            waitSeconds(kScoreDelaySeconds).andThen(() -> hasCoral = false)
        ).withName(getMethodInfo());
    }
}
