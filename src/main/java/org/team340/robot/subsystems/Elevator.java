package org.team340.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.RobotMap;

public class Elevator extends GRRSubsystem {

    private final TalonFX leadMotor;
    private final TalonFX followMotor;
    private final CANcoder leadCanCoder;
    private final CANcoder followCanCoder;

    public Elevator() {
        // MOTOR SETUP
        leadMotor = new TalonFX(RobotMap.kElevatorLead, RobotMap.kLowerCANBus);
        followMotor = new TalonFX(RobotMap.kElevatorFollow, RobotMap.kLowerCANBus);

        TalonFXConfiguration leadCfg = new TalonFXConfiguration();
        TalonFXConfiguration followCfg = new TalonFXConfiguration();
        // TODO: FILL IN CFG
        PhoenixUtil.run("Clear Lead Motor Sticky Faults", leadMotor, () -> leadMotor.clearStickyFaults());
        PhoenixUtil.run("Clear Follow Motor Sticky Faults", followMotor, () -> followMotor.clearStickyFaults());
        PhoenixUtil.run("Apply Lead TalonFXConfiguration", leadMotor, () -> leadMotor.getConfigurator().apply(leadCfg));
        PhoenixUtil.run("Apply Follow TalonFXConfiguration", followMotor, () ->
            followMotor.getConfigurator().apply(followCfg)
        );

        followMotor.setControl(new Follower(leadMotor.getDeviceID(), false));

        // CAN CODER SETUP
        leadCanCoder = new CANcoder(RobotMap.kElevatorLeadEncoder, RobotMap.kLowerCANBus);
        followCanCoder = new CANcoder(RobotMap.kElevatorFollowEncoder, RobotMap.kLowerCANBus);

        CANcoderConfiguration leadCanCoderConfig = new CANcoderConfiguration();
        CANcoderConfiguration followCanCoderConfig = new CANcoderConfiguration();
        //: TODO FILL IN CFG
        PhoenixUtil.run("Clear Lead Sticky Faults", leadCanCoder, () -> leadCanCoder.clearStickyFaults());
        PhoenixUtil.run("Apply Lead CANcoderConfiguration", leadCanCoder, () ->
            leadCanCoder.getConfigurator().apply(leadCanCoderConfig)
        );
        PhoenixUtil.run("Clear Follow Sticky Faults", followCanCoder, () -> followCanCoder.clearStickyFaults());
        PhoenixUtil.run("Apply Follow CANcoderConfiguration", followCanCoder, () ->
            followCanCoder.getConfigurator().apply(followCanCoderConfig)
        );
    }
}
