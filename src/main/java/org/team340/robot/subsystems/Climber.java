package org.team340.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants;
import org.team340.robot.Constants.UpperCAN;

public final class Climber extends GRRSubsystem {

    public static enum ClimberPosition {
        kStore(0.0),
        kDeploy(0.0),
        kClimb(0.0);

        private final TunableDouble angle;

        private ClimberPosition(double angle) {
            this.angle = Tunable.doubleValue("climber/positions/" + name(), angle);
        }

        private double rotations() {
            return angle.value();
        }
    }

    private final TalonFX motor;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;

    public Climber() {
        motor = new TalonFX(UpperCAN.kClimberMotor);

        position = motor.getPosition();
        velocity = motor.getVelocity();

        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0;

        // TODO: add configuration.

        PhoenixUtil.run("Apply Climber Motor Configuration", () -> motor.getConfigurator().apply(config));

        PhoenixUtil.run("Set Climber Motor Frequency", () ->
            BaseStatusSignal.setUpdateFrequencyForAll(20, position, velocity)
        );

        PhoenixUtil.run("Optimize Climber CAN Utilization", () -> ParentDevice.optimizeBusUtilizationForAll(5, motor));
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(position, velocity);
    }

    // *************** Helper Functions ***************

    /**
     * Stops the climber.
     */
    private void stop() {
        motor.stopMotor();
    }

    /**
     * Sets the output voltage of the motor.
     * @param percentOutput The percent of the output voltage to set
     */
    private void setTargetSpeed(double percentOutput) {
        if (percentOutput < 0.0) {
            stop();
            DriverStation.reportWarning("The climber motor cannot be set to negative speeds.", false);
            return;
        }

        motor.setVoltage(percentOutput * Constants.kVoltage);
    }

    /**
     * Gets the number of rotations of the climber's encoder.
     * @return The position in rotations.
     */
    private double getRotations() {
        return BaseStatusSignal.getLatencyCompensatedValueAsDouble(position, velocity);
    }

    // *************** Commands ***************

    /**
     * Moves the climber forward until it goes passed the current value of the supplier.
     * @param rotationsSupplier The supplier of the possition that must be surpassed.
     */
    private Command goForwardToPosition(DoubleSupplier rotationsSupplier) {
        return commandBuilder("Climber.goForwardToPosition()")
            .onExecute(() -> setTargetSpeed(1.0))
            .isFinished(() -> getRotations() > rotationsSupplier.getAsDouble())
            .onEnd(this::stop);
    }

    /**
     * Deploys the climber.
     */
    public Command deploy() {
        return goForwardToPosition(ClimberPosition.kDeploy::rotations).withName("Climber.deploy()");
    }

    /**
     * Makes the climber go to the climb position.
     */
    public Command climb() {
        return goForwardToPosition(ClimberPosition.kClimb::rotations).withName("Climber.climb()");
    }
}
