package org.team340.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.RobotMap;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;

@Logged
public class GooseneckPivot extends GRRSubsystem {
    private static final TunableDouble kDeployedPosition = Tunable.doubleValue("GooseneckPivot/kDeployedPosition", 0.0);
    private static final TunableDouble kSafePosition = Tunable.doubleValue("GooseneckPivot/kSafePosition", 0.0);
    
    private final TalonFX pivotMotor;
    private final CANdi pivotEncoder;

    public GooseneckPivot() {
        pivotMotor = new TalonFX(RobotMap.kGooseneckPivotMotor);
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        
        PhoenixUtil.run("Clear pivotMotor sticky faults", pivotMotor, () -> pivotMotor.clearStickyFaults());
        PhoenixUtil.run("Apply pivotMotor TalonFXConfiguration", pivotMotor, () -> pivotMotor.getConfigurator().apply(pivotConfig));

        pivotEncoder = new CANdi(RobotMap.kGooseneckPivotEncoder);

        final HardwareLimitSwitchConfigs limitConfigs = new HardwareLimitSwitchConfigs();
        limitConfigs.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANcoder;
        limitConfigs.ForwardLimitRemoteSensorID = pivotEncoder.getDeviceID();

        PhoenixUtil.run("Apply pivotMotor HardwareLimitSwitchConfigs", pivotMotor, () -> pivotMotor.getConfigurator().apply(limitConfigs));
    }

    private void applyPosition(double position) {
        final double kInverseTwoPi = 1 / (Math.PI * 2);
        pivotMotor.setPosition(position * kInverseTwoPi);
    }

    private void stop() {
        pivotMotor.stopMotor();
    }

    public Command goToPosition(DoubleSupplier position) {
        return commandBuilder("GooseneckPivot.goToPosition(supplier)")
            .onExecute(() -> applyPosition(position.getAsDouble()))
            .onEnd(this::stop);
    }

    public Command goToPosition(double position) {
        return goToPosition(() -> position);
    }

    public Command deploy() {
        return goToPosition(kDeployedPosition::value);
    }

    public Command retract() {
        return goToPosition(kSafePosition::value);
    }
}