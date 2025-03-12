package org.team340.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.UpperCAN;

@Logged
public final class Climber extends GRRSubsystem {

    private static enum ClimberPosition {
        kDeploy(94.0),
        kClimb(420.0);

        private final TunableDouble rotations;

        private ClimberPosition(double rotations) {
            this.rotations = Tunable.doubleValue("climber/positions/" + name(), rotations);
        }

        private double rotations() {
            return rotations.value();
        }
    }

    private static final TunableDouble kVoltageOut = Tunable.doubleValue("climber/kVoltageOut", 12.0);

    private final TalonFX motor;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;

    private final VoltageOut voltageControl;

    public Climber() {
        motor = new TalonFX(UpperCAN.kClimberMotor);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 100.0;
        config.CurrentLimits.SupplyCurrentLimit = 70.0;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 425.0;

        PhoenixUtil.run("Clear Climber Sticky Faults", () -> motor.clearStickyFaults());
        PhoenixUtil.run("Apply Climber Motor Configuration", () -> motor.getConfigurator().apply(config));

        position = motor.getPosition();
        velocity = motor.getVelocity();

        PhoenixUtil.run("Set Climber Signal Frequencies", () ->
            BaseStatusSignal.setUpdateFrequencyForAll(100, position, velocity)
        );
        PhoenixUtil.run("Optimize Climber CAN Utilization", () -> ParentDevice.optimizeBusUtilizationForAll(10, motor));

        voltageControl = new VoltageOut(0.0);
        voltageControl.EnableFOC = true;
        voltageControl.UpdateFreqHz = 0.0;

        PhoenixUtil.run("Zero Climber Position", () -> motor.setPosition(0.0));
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(position, velocity);
    }

    // *************** Helper Functions ***************

    /**
     * Gets the number of rotations of the climber's encoder.
     * @return The position in rotations.
     */
    private double getPosition() {
        return BaseStatusSignal.getLatencyCompensatedValueAsDouble(position, velocity);
    }

    /**
     * Checks if the climber is deployed.
     * @return True if the climber is deployed, false otherwise.
     */
    public boolean isDeployed() {
        return getPosition() > ClimberPosition.kDeploy.rotations();
    }

    // *************** Commands ***************

    /**
     * Deploys the climber.
     */
    public Command deploy() {
        return goTo(ClimberPosition.kDeploy).withName("Climber.deploy()");
    }

    /**
     * Makes the climber go to the climb position.
     */
    public Command climb() {
        return goTo(ClimberPosition.kClimb).withName("Climber.climb()");
    }

    /**
     * Goes to a position.
     * @param position The position to go to.
     */
    private Command goTo(ClimberPosition position) {
        return commandBuilder("Climber.goTo(" + position.name() + ")")
            .onExecute(() -> motor.setControl(voltageControl.withOutput(kVoltageOut.value())))
            .isFinished(() -> getPosition() >= position.rotations())
            .onEnd(motor::stopMotor);
    }

    public Command coastMode() {
        MotorOutputConfigs config = new MotorOutputConfigs();

        return commandBuilder("Climber.coastMode()")
            .onInitialize(() -> {
                motor.getConfigurator().refresh(config);
                config.NeutralMode = NeutralModeValue.Coast;
                motor.getConfigurator().apply(config);
            })
            .onEnd(() -> {
                motor.getConfigurator().refresh(config);
                config.NeutralMode = NeutralModeValue.Brake;
                motor.getConfigurator().apply(config);
                PhoenixUtil.run("Set Climber Zero", () -> motor.setPosition(0.0));
            })
            .ignoringDisable(true);
    }
}
