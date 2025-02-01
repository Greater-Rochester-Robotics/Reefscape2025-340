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
import org.team340.robot.Constants.RobotMap;

@Logged
public class Intake extends GRRSubsystem {

    private static final TunableDouble kIntakingSpeed = Tunable.doubleValue("Intake/kIntakingSpeed", 0.0);

    private final TalonFX intakeMotor;
    private final DigitalInput beamBreak;

    public Intake() {
        intakeMotor = new TalonFX(RobotMap.kIntakeMotor);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 0.0;
        config.CurrentLimits.SupplyCurrentLimit = 0.0;

        PhoenixUtil.run("Clear Intake Motor Sticky Faults", intakeMotor, () -> intakeMotor.clearStickyFaults());
        PhoenixUtil.run("Apply Intake Motor TalonFXConfiguration", intakeMotor, () ->
            intakeMotor.getConfigurator().apply(config)
        );

        beamBreak = new DigitalInput(RobotMap.kIntakeBeamBreak);
    }

    // *************** Helper Functions ***************

    /**
     * Stops the intake.
     */
    private void stop() {
        intakeMotor.stopMotor();
    }

    /**
     * Sets the target speed of the intake wheels.
     * @param speed The target speed. Speed should be between 1.0 and -1.0.
     */
    private void setTargetSpeed(double speed) {
        intakeMotor.set(speed);
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
        return commandBuilder().onExecute(() -> setTargetSpeed(speedSupplier.getAsDouble())).onEnd(this::stop);
    }

    /**
     * Runs the intake at {@code speed}.
     * @param speed The speed to run the intake at, which should be between 1.0 and -1.0.
     */
    private Command runAtSpeed(double speed) {
        return runAtSpeed(() -> speed);
    }

    /**
     * Runs the intake at the {@link Intake#kIntakingSpeed kIntakingSpeed}.
     */
    public Command intake() {
        return runAtSpeed(kIntakingSpeed::value);
    }
}
