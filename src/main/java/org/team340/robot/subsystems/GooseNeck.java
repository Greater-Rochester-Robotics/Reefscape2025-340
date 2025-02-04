package org.team340.robot.subsystems;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.RobotMap;

@Logged
public class GooseNeck extends GRRSubsystem {

    public enum Positions {
        kIn(0.0),
        kScoreForward(0.0),
        kScoreLeft(0.0),
        kScoreRight(0.0);

        private TunableDouble position;

        Positions(double position) {
            this.position = Tunable.doubleValue(
                getClass().getEnclosingClass().getSimpleName() + "/" + getClass().getSimpleName() + "/" + name(),
                position
            );
        }

        public double getPosition() {
            return position.value();
        }
    }

    // These are intentionally not tunable.
    private static final double kUpperLimit = 0.0;
    private static final double kLowerLimit = 0.0;

    private final TalonFX pivotMotor;

    public GooseNeck() {
        pivotMotor = new TalonFX(RobotMap.kGooseNeckMotor);
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

        // TODO fix config

        PhoenixUtil.run("Clear pivotMotor sticky faults", pivotMotor, () -> pivotMotor.clearStickyFaults());
        PhoenixUtil.run("Apply pivotMotor TalonFXConfiguration", pivotMotor, () ->
            pivotMotor.getConfigurator().apply(pivotConfig)
        );

        final HardwareLimitSwitchConfigs limitConfigs = new HardwareLimitSwitchConfigs();
        limitConfigs.ForwardLimitRemoteSensorID = RobotMap.kGooseCANdi;
        limitConfigs.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANdiS2;

        PhoenixUtil.run("Apply pivotMotor HardwareLimitSwitchConfigs", pivotMotor, () ->
            pivotMotor.getConfigurator().apply(limitConfigs)
        );
    }

    // *************** Helper Functions ***************

    /**
     * Stops the pivot motor. Should be run at the onEnd of commands.
     */
    private void stop() {
        pivotMotor.stopMotor();
    }

    /**
     * Sets the target position of the pivot.
     * @param position The position to target in radians.
     */
    private void setTargetPosition(double position) {
        if (position > kUpperLimit || position < kLowerLimit) {
            DriverStation.reportWarning(
                "The " +
                getName() +
                " position " +
                position +
                " must be less than " +
                kUpperLimit +
                " and greater than " +
                kLowerLimit +
                ".",
                false
            );
            return;
        }

        final double kInverseTwoPi = 1 / (Math.PI * 2);
        pivotMotor.setPosition(position * kInverseTwoPi);
    }

    // *************** Commands ***************

    /**
     * Moves the pivot to the position supplied by {@code positionSupplier}.
     * @param positionSupplier The supplier of the position. Positions should be in radians.
     */
    private Command goToPosition(DoubleSupplier positionSupplier) {
        return commandBuilder(getMethodInfo("supplier"))
            .onExecute(() -> setTargetPosition(positionSupplier.getAsDouble()))
            .onEnd(this::stop);
    }

    /**
     * Moves the pivot to the {@code position}.
     * @param position The position to move the pivot to. The position should be in radians.
     */
    private Command goToPosition(double position) {
        return goToPosition(() -> position).withName(getMethodInfo(String.valueOf(position)));
    }

    /**
     * Moves the pivot to the {@code position}.
     * @param position The position to move the pivot to.
     */
    public Command goToPosition(Positions position) {
        return goToPosition(position::getPosition).withName(getMethodInfo(position.name()));
    }
}
