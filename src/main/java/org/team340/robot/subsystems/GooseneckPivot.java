package org.team340.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.team340.lib.util.command.GRRSubsystem;
import org.team340.robot.Constants.RobotMap;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;

import edu.wpi.first.wpilibj2.command.Command;

public class GooseneckPivot extends GRRSubsystem {
    final TalonFX pivotMotor;
    final CANdi pivotEncoder;

    public GooseneckPivot() {
        pivotMotor = new TalonFX(RobotMap.kGooseneckPivotMotor);
        pivotEncoder = new CANdi(RobotMap.kGooseneckPivotEncoder);

        final HardwareLimitSwitchConfigs limitConfigs = new HardwareLimitSwitchConfigs();
        limitConfigs.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANcoder;
        limitConfigs.ForwardLimitRemoteSensorID = pivotEncoder.getDeviceID();

        pivotMotor.getConfigurator().apply(limitConfigs);
    }

    public void applyPosition(double position) {
        final double kInverseTwoPi = 1 / (Math.PI * 2);
        pivotMotor.setPosition(position * kInverseTwoPi);
    }

    public void stop() {
        pivotMotor.stopMotor();
    }

    public Command goToPosition(DoubleSupplier position) {
        return commandBuilder("GooseneckPivot.goToPosition(supplier)")
            .onExecute(() -> applyPosition(position.getAsDouble()))
            .onEnd(this::stop);
    }
}