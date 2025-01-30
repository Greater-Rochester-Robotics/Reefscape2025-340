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

        // TODO Add configuration.
        TalonFXConfiguration config = new TalonFXConfiguration();

        PhoenixUtil.run("Clear Intake Motor Sticky Faults", intakeMotor, () -> intakeMotor.clearStickyFaults());
        PhoenixUtil.run("Apply Intake Motor TalonFXConfiguration", intakeMotor, () ->
            intakeMotor.getConfigurator().apply(config)
        );

        beamBreak = new DigitalInput(RobotMap.kIntakeBeamBreak);
    }

    private  void setTargetSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public boolean coralDetected() {
        return beamBreak.get();
    }

    private Command runAtSpeed(DoubleSupplier speed) {
        return commandBuilder()
            .onExecute(() -> setTargetSpeed(speed.getAsDouble()))
            .onEnd(() -> intakeMotor.stopMotor());
    }

    public Command runAtSpeed(double speed) {
        return runAtSpeed(() -> speed);
    }

    public Command intake() {
        return runAtSpeed(kIntakingSpeed::value);
    }
}
