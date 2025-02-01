package org.team340.robot.subsystems;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
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
public class GooseneckPivot extends GRRSubsystem {

    private static final TunableDouble kDeployedPosition = Tunable.doubleValue("GooseneckPivot/kDeployedPosition", 0.0);
    private static final TunableDouble kSafePosition = Tunable.doubleValue("GooseneckPivot/kSafePosition", 0.0);

    // These are intentionally not tunable.
    private static final double kUpperLimit = 0.0;
    private static final double kLowerLimit = 0.0;

    private final TalonFX pivotMotor;
    private final CANdi pivotEncoder;

    public GooseneckPivot() {
        pivotMotor = new TalonFX(RobotMap.kGooseneckPivotMotor);
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

        PhoenixUtil.run("Clear pivotMotor sticky faults", pivotMotor, () -> pivotMotor.clearStickyFaults());
        PhoenixUtil.run("Apply pivotMotor TalonFXConfiguration", pivotMotor, () ->
            pivotMotor.getConfigurator().apply(pivotConfig)
        );

        pivotEncoder = new CANdi(RobotMap.kGooseneckPivotEncoder);

        final HardwareLimitSwitchConfigs limitConfigs = new HardwareLimitSwitchConfigs();
        limitConfigs.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANcoder;
        limitConfigs.ForwardLimitRemoteSensorID = pivotEncoder.getDeviceID();

        PhoenixUtil.run("Apply pivotMotor HardwareLimitSwitchConfigs", pivotMotor, () ->
            pivotMotor.getConfigurator().apply(limitConfigs)
        );
    }

    /**
     * Sets the target position of the pivot.
     * @param position The position to target in radians.
     */
    private void setTargetPosition(double position) {
        if (position > kUpperLimit || position < kLowerLimit) {
            DriverStation.reportWarning(
                "The GooseneckPivot position " +
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

    /**
     * Stops the pivot motor. Should be run at the onEnd of commands.
     */
    private void stop() {
        pivotMotor.stopMotor();
    }

    /**
     * Moves the pivot to the position supplied by {@code positionSupplier}.
     * @param positionSupplier The supplier of the position. Positions should be in radians.
     */
    public Command goToPosition(DoubleSupplier positionSupplier) {
        return commandBuilder("GooseneckPivot.goToPosition(supplier)")
            .onExecute(() -> setTargetPosition(positionSupplier.getAsDouble()))
            .onEnd(this::stop);
    }

    /**
     * Moves the pivot to the {@code position}.
     * @param position The position to move the pivot to. The position should be in radians.
     */
    public Command goToPosition(double position) {
        return goToPosition(() -> position).withName("GooseneckPivot.goToPosition(" + position + ")");
    }

    /**
     * Moves the pivot to the {@link GooseneckPivot#kDeployedPosition kDeployedPosition}.
     */
    public Command deploy() {
        return goToPosition(kDeployedPosition::value).withName("GooseneckPivot.deploy()");
    }

    /**
     * Moves the pivot to the {@link GooseneckPivot#kSafePosition}.
     */
    public Command retract() {
        return goToPosition(kSafePosition::value).withName("GooseneckPivot.retract()");
    }
}
