package org.team340.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants;
import org.team340.robot.Constants.DIO;
import org.team340.robot.Constants.UpperCAN;

@Logged
public class Intake extends GRRSubsystem {

    private static final TunableDouble kIntakingSpeed = Tunable.doubleValue(
        getSubsystemName() + "/kIntakingSpeed",
        0.0
    );

    private final TalonFX motor;
    private final DigitalInput beamBreak;

    public Intake() {
        motor = new TalonFX(UpperCAN.kIntakeMotor);
        beamBreak = new DigitalInput(DIO.kIntakeBeamBreak);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimit = 30.0;

        PhoenixUtil.run("Clear Intake Motor Sticky Faults", motor, () -> motor.clearStickyFaults());
        PhoenixUtil.run("Apply Intake Motor TalonFXConfiguration", motor, () -> motor.getConfigurator().apply(config));
    }

    // *************** Helper Functions ***************

    /**
     * Stops the intake.
     */
    private void stop() {
        motor.stopMotor();
    }

    /**
     * Sets the target speed of the intake wheels.
     * @param speed The target speed. Speed should be between 1.0 and -1.0.
     */
    private void setTargetSpeed(double speed) {
        motor.setVoltage(speed * Constants.kVoltage);
    }

    /**
     * Returns whether the beam break sees the coral or not.
     * @return True if the beam break detects an object, false otherwise.
     */
    public boolean coralDetected() {
        return beamBreak.get();
    }

    // *************** Commands ***************

    /**
     * Runs the intake at the speed supplied by the {@code speedSupplier}.
     * @param speedSupplier Provides the speed the intake will be run at, which should be between 1.0 and -1.0.
     */
    private Command runAtSpeed(DoubleSupplier speedSupplier) {
        return commandBuilder("supplier")
            .onExecute(() -> setTargetSpeed(speedSupplier.getAsDouble()))
            .onEnd(this::stop);
    }

    /**
     * Runs the intake at the {@link Intake#kIntakingSpeed kIntakingSpeed}.
     */
    public Command intake() {
        return runAtSpeed(kIntakingSpeed::value).withName(getMethodInfo());
    }
}
