package org.team340.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import edu.wpi.first.wpilibj2.command.Command;

import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.DIO;

public class Climber extends GRRSubsystem {
 	public static enum ClimberPosition {
        kStore(0.0),
        kOut(0.0),
        kClimb(0.0);

        private final TunableDouble kAngle;

        private ClimberPosition(final double angle) {
            kAngle = Tunable.doubleValue(getEnumName(this), angle);
        }

        private double getAngle() {
            return kAngle.value();
        }
    }

    private final TalonFX climberMotor;
    private final MotionMagicVoltage climberMotorController;
    private final CANcoder climberEncoder;


    public Climber() {
		
		climberMotor = new TalonFX(DIO.kClimberMotor);
        climberMotorController = new MotionMagicVoltage(0.0);
		climberEncoder = new CANcoder (DIO.kClimberEncoder);

	   TalonFXConfiguration motorCfg = new TalonFXConfiguration();

        motorCfg.CurrentLimits.StatorCurrentLimit = 80;
        motorCfg.CurrentLimits.SupplyCurrentLimit = 100;

        motorCfg.Slot0.kP = 0;
        motorCfg.Slot0.kI = 0;
        motorCfg.Slot0.kD = 0;
        motorCfg.Slot0.kG = 0;
        motorCfg.Slot0.kS = 0;
        motorCfg.Slot0.kV = 0;

        motorCfg.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANdiS1;
        motorCfg.HardwareLimitSwitch.ReverseLimitRemoteSensorID = 22;
        motorCfg.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        motorCfg.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        motorCfg.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;

        PhoenixUtil.run("Clear Lead Motor Sticky Faults", climberMotor, () -> climberMotor.clearStickyFaults());

        PhoenixUtil.run("Apply Lead TalonFXConfiguration", climberMotor, () -> climberMotor.getConfigurator().apply(motorCfg)
        );
       
    }

    // *************** Helper Functions ***************

    /**
     * Stops the climber.
     */
    private void stop() {
        climberMotor.stopMotor();
    }

    /**
     * Sets the climber motor to target the specified position.
     * @param position The position to target.
     */
    private void setTargetPosition(double position) {
        climberMotor.setControl(climberMotorController.withPosition(position));
    }

    // *************** Commands ***************

    public Command climb () {
		return commandBuilder()
		//.onExecute(() -> {
			// Setting lead motor also sets the follow motor
			//climberMotor.setPosition(0)(positionControl.withPosition(position.rotations()));
	//	})
		//.isFinished(() -> isAtPosition(position))
		.onEnd(this::stop);
    }
}
