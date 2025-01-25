package org.team340.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.RobotMap;

public class Elevator extends GRRSubsystem {

    private final TalonFX leadMotor;
    private final TalonFX followMotor;

    public Elevator() {
        leadMotor = new TalonFX(RobotMap.kElevatorLead, RobotMap.kLowerCANBus);
        followMotor = new TalonFX(RobotMap.kElevatorFollow, RobotMap.kLowerCANBus);

        TalonFXConfiguration leadCfg = new TalonFXConfiguration();
        TalonFXConfiguration followCfg = new TalonFXConfiguration();
        // TODO: FILL IN CFG
        PhoenixUtil.run("Apply TalonFXConfiguration", leadMotor, () -> leadMotor.getConfigurator().apply(leadCfg));
        PhoenixUtil.run("Apply TalonFXConfiguration", followMotor, () -> followMotor.getConfigurator().apply(followCfg)
        );
        followMotor.setControl(new Follower(leadMotor.getDeviceID(), false));
    }
}
