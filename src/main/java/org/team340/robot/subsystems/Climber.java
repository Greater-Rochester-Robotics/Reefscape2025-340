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
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.RioCAN;

@Logged
public final class Climber extends GRRSubsystem {

    private static final TunableTable tunables = Tunables.getTable("climber");

    private static final TunableDouble volts = tunables.value("volts", 12.0);
    private static final TunableDouble overrideVolts = tunables.value("overrideVolts", 4.0);

    private static enum ClimberPosition {
        DEPLOY(90.0),
        CLIMB(412.0);

        private final TunableDouble rotations;

        private ClimberPosition(double rotations) {
            this.rotations = tunables.value("positions/" + name(), rotations);
        }

        private double rotations() {
            return rotations.get();
        }
    }

    private final TalonFX motor;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;

    private final VoltageOut voltageControl;

    public Climber() {
        motor = new TalonFX(RioCAN.CLIMBER_MOTOR);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 100.0;
        config.CurrentLimits.SupplyCurrentLimit = 70.0;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 450.0;

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
        return getPosition() >= ClimberPosition.DEPLOY.rotations();
    }

    /**
     * Checks if the climber has climbed.
     * @return True if the climber has climbed, false otherwise.
     */
    public boolean isClimbed() {
        return getPosition() >= ClimberPosition.DEPLOY.rotations();
    }

    // *************** Commands ***************

    /**
     * Deploys the climber.
     */
    public Command deploy() {
        return goTo(ClimberPosition.DEPLOY).withName("Climber.deploy()");
    }

    /**
     * Makes the climber go to the climb position.
     */
    public Command climb() {
        return goTo(ClimberPosition.CLIMB).onlyIf(this::isDeployed).withName("Climber.climb()");
    }

    /**
     * Overrides the climber to go BEYOND
     */
    public Command override() {
        return commandBuilder()
            .onExecute(() -> motor.setControl(voltageControl.withOutput(overrideVolts.get())))
            .onEnd(motor::stopMotor)
            .onlyIf(this::isClimbed)
            .withName("Climber.override()");
    }

    /**
     * Goes to a position.
     * @param position The position to go to.
     */
    private Command goTo(ClimberPosition position) {
        return commandBuilder("Climber.goTo(" + position.name() + ")")
            .onExecute(() -> motor.setControl(voltageControl.withOutput(volts.get())))
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
