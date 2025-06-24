package org.team340.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.CAN0;

@Logged
public final class Intake extends GRRSubsystem {

    private static final TunableTable tunables = Tunables.getTable("intake");

    private static final TunableDouble intakeVolts = tunables.value("intakeVolts", 6.0);
    private static final TunableDouble barfVolts = tunables.value("barfVolts", 7.0);
    private static final TunableDouble swallowVolts = tunables.value("swallowVolts", -6.0);
    private static final TunableDouble currentThreshold = tunables.value("currentThreshold", 30.0);
    private static final TunableDouble unjamTime = tunables.value("unjamTime", 0.2);

    private final TalonFX motor;
    private final CANrange canRange;

    private final StatusSignal<Current> current;
    private final StatusSignal<Boolean> detected;

    private final VoltageOut voltageControl;

    private boolean unjamming = false;

    public Intake() {
        motor = new TalonFX(CAN0.INTAKE_MOTOR, CAN0.ID);
        canRange = new CANrange(CAN0.INTAKE_CANRANGE, CAN0.ID);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 30.0;

        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        CANrangeConfiguration canRangeConfig = new CANrangeConfiguration();

        canRangeConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 5000.0;
        canRangeConfig.ProximityParams.ProximityThreshold = 3.5;
        canRangeConfig.ProximityParams.ProximityHysteresis = 0.02;

        canRangeConfig.FovParams.FOVRangeX = 10.0;
        canRangeConfig.FovParams.FOVRangeY = 10.0;

        canRangeConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRangeUserFreq;
        canRangeConfig.ToFParams.UpdateFrequency = 50.0;

        PhoenixUtil.run("Clear Intake Motor Sticky Faults", () -> motor.clearStickyFaults());
        PhoenixUtil.run("Clear Intake CANrange Sticky Faults", () -> canRange.clearStickyFaults());
        PhoenixUtil.run("Apply Intake Motor TalonFXConfiguration", () -> motor.getConfigurator().apply(motorConfig));
        PhoenixUtil.run("Apply Intake CANrange CANrangeConfiguration", () ->
            canRange.getConfigurator().apply(canRangeConfig)
        );

        current = motor.getStatorCurrent();
        detected = canRange.getIsDetected();

        PhoenixUtil.run("Set Intake Signal Frequencies", () ->
            BaseStatusSignal.setUpdateFrequencyForAll(100, current, detected)
        );
        PhoenixUtil.run("Optimize Intake CAN Utilization", () ->
            ParentDevice.optimizeBusUtilizationForAll(10, motor, canRange)
        );

        voltageControl = new VoltageOut(0.0);
        voltageControl.EnableFOC = false;
        voltageControl.UpdateFreqHz = 0.0;
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(current, detected);
    }

    @NotLogged
    public boolean unjamming() {
        return unjamming;
    }

    public boolean coralDetected() {
        return detected.getValue();
    }

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
                if (debouncer.calculate(current.getValueAsDouble() > currentThreshold.get())) {
                    unjamTimer.start();
                }

                if ((unjamTimer.isRunning() && !unjamTimer.hasElapsed(unjamTime.get())) || swallow.getAsBoolean()) {
                    motor.setControl(voltageControl.withOutput(swallowVolts.get()));
                } else {
                    motor.setControl(voltageControl.withOutput(intakeVolts.get()));
                    unjamTimer.stop();
                    unjamTimer.reset();
                }

                unjamming = unjamTimer.isRunning();
            })
            .onEnd(() -> {
                motor.stopMotor();
                unjamming = false;
            });
    }

    /**
     * Sets the intake to barf.
     */
    public Command barf() {
        return commandBuilder("Intake.barf()")
            .onExecute(() -> motor.setControl(voltageControl.withOutput(barfVolts.get())))
            .onEnd(motor::stopMotor);
    }

    /**
     * Sets the intake to swallow.
     */
    public Command swallow() {
        return commandBuilder("Intake.swallow()")
            .onExecute(() -> motor.setControl(voltageControl.withOutput(swallowVolts.get())))
            .onEnd(motor::stopMotor);
    }
}
