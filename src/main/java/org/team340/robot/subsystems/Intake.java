package org.team340.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.RobotMap;

public class Intake extends GRRSubsystem {

    final double kIntakingSpeed = 0.0;
    final double kBarfingSpeed = 0.0;

    final TalonFX intakeMotor;
    final DigitalInput beamBreak;

    public Intake() {
        intakeMotor = new TalonFX(RobotMap.kIntakeMotor);

        TalonFXConfiguration config = new TalonFXConfiguration();

        PhoenixUtil.run("Clear Intake Motor Sticky Faults", intakeMotor, () -> intakeMotor.clearStickyFaults());
        PhoenixUtil.run("Apply Intake Motor TalonFXConfiguration", intakeMotor, () ->
            intakeMotor.getConfigurator().apply(config)
        );

        beamBreak = new DigitalInput(RobotMap.kIntakeBeamBreak);
    }

    public void setTargetSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public boolean hasPiece() {
        return beamBreak.get();
    }

    public Command runAtSpeed(DoubleSupplier speed) {
        return commandBuilder()
            .onExecute(() -> intakeMotor.set(speed.getAsDouble()))
            .onEnd(() -> intakeMotor.stopMotor());
    }

    public Command runAtSpeed(double speed) {
        return runAtSpeed(() -> speed);
    }

    public Command intake() {
        return runAtSpeed(kIntakingSpeed);
    }

    public Command barf() {
        return runAtSpeed(kBarfingSpeed);
    }
}
