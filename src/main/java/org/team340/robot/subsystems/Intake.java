package org.team340.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.UpperCAN;

@Logged
public final class Intake extends GRRSubsystem {

    private static final TunableDouble kIntakeVoltage = Tunable.doubleValue("intake/kIntakeVoltage", 6.0);
    private static final TunableDouble kBarfVoltage = Tunable.doubleValue("intake/kBarfVoltage", 7.0);
    private static final TunableDouble kSwallowVoltage = Tunable.doubleValue("intake/kSwallowVoltage", -6.0);
    private static final TunableDouble kCurrentThreshold = Tunable.doubleValue("intake/kCurrentThreshold", 24.0);
    private static final TunableDouble kUnjamTime = Tunable.doubleValue("intake/kUnjamTime", 0.2);

    private final TalonFX motor;

    private final StatusSignal<Current> current;

    private final VoltageOut voltageControl;

    public Intake() {
        motor = new TalonFX(UpperCAN.kIntakeMotor);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimit = 30.0;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        PhoenixUtil.run("Clear Intake Motor Sticky Faults", () -> motor.clearStickyFaults());
        PhoenixUtil.run("Apply Intake Motor TalonFXConfiguration", () -> motor.getConfigurator().apply(config));

        current = motor.getStatorCurrent();

        PhoenixUtil.run("Set Intake Signal Frequencies", () -> BaseStatusSignal.setUpdateFrequencyForAll(100, current));
        PhoenixUtil.run("Optimize Intake CAN Utilization", () -> ParentDevice.optimizeBusUtilizationForAll(10, motor));

        voltageControl = new VoltageOut(0.0);
        voltageControl.EnableFOC = false;
        voltageControl.UpdateFreqHz = 0.0;
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(current);
    }

    // *************** Commands ***************

    /**
     * Runs the intake.
     */
    public Command intake(BooleanSupplier swallow) {
        Debouncer debouncer = new Debouncer(0.2);
        Timer unjamTimer = new Timer();

        return commandBuilder("Intake.intake()")
            .onInitialize(() -> {
                debouncer.calculate(false);
                unjamTimer.stop();
                unjamTimer.reset();
            })
            .onExecute(() -> {
                if (debouncer.calculate(current.getValueAsDouble() > kCurrentThreshold.value())) {
                    unjamTimer.start();
                }

                if ((unjamTimer.isRunning() && !unjamTimer.hasElapsed(kUnjamTime.value())) || swallow.getAsBoolean()) {
                    motor.setControl(voltageControl.withOutput(kSwallowVoltage.value()));
                } else {
                    motor.setControl(voltageControl.withOutput(kIntakeVoltage.value()));
                    unjamTimer.stop();
                    unjamTimer.reset();
                }
            })
            .onEnd(motor::stopMotor);
    }

    /**
     * Sets the intake to barf.
     */
    public Command barf() {
        return commandBuilder("Intake.barf()")
            .onExecute(() -> motor.setControl(voltageControl.withOutput(kBarfVoltage.value())))
            .onEnd(motor::stopMotor);
    }

    /**
     * Sets the intake to swallow.
     */
    public Command swallow() {
        return commandBuilder("Intake.swallow()")
            .onExecute(() -> motor.setControl(voltageControl.withOutput(kSwallowVoltage.value())))
            .onEnd(motor::stopMotor);
    }
}
