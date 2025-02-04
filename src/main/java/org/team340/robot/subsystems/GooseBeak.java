package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.RobotMap;

/**
 * Controls the rollers and sensors used to score the coral, not including the pivot.
 */
@Logged
public class GooseBeak extends GRRSubsystem {

    public enum Speeds {
        kIntake(0.0),
        kScore(0.0),
        kIndexing(0.0);

        private TunableDouble speed;

        Speeds(double speed) {
            this.speed = Tunable.doubleValue(
                getClass().getEnclosingClass().getSimpleName() + "/" + getClass().getSimpleName() + "/" + name(),
                speed
            );
        }

        public double getSpeed() {
            return speed.value();
        }
    }

    private final TalonFXS rollerMotor;

    public GooseBeak() {
        rollerMotor = new TalonFXS(RobotMap.kGooseBeakMotor);

        TalonFXSConfiguration rollerConfig = new TalonFXSConfiguration();

        rollerConfig.CurrentLimits.StatorCurrentLimit = 0.0;
        rollerConfig.CurrentLimits.SupplyCurrentLimit = 0.0;

        rollerConfig.HardwareLimitSwitch.ReverseLimitSource = RobotMap.kGooseBeamBreak;
        rollerConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = RobotMap.kGooseCANdi;
        rollerConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;

        PhoenixUtil.run("Apply the TalonFXSConfiguration to the rollerMotor", rollerMotor, () ->
            rollerMotor.getConfigurator().apply(rollerConfig)
        );
    }

    // *************** Helper Functions ***************

    /**
     * Stops the roller motor.
     */
    private void stop() {
        rollerMotor.stopMotor();
    }

    /**
     * Sets the target speed of the rollers.
     * @param speed The target speed. Speeds should be between 1.0 and -1.0.
     */
    private void setTargetSpeed(double speed) {
        rollerMotor.set(speed);
    }

    /**
     * Checks if the beam break detects an object.
     * @return True if the beam break detects an object, false otherwise.
     */
    public boolean hasPiece() {
        return rollerMotor.getReverseLimit().getValue().equals(ReverseLimitValue.ClosedToGround);
    }

    // *************** Commands ***************

    /**
     * Runs the rollers at the speed supplied by {@code speedSupplier}.
     * @param speedSupplier Supplies the speed the rollers are run at. Speeds should be between 1.0 and -1.0
     */
    private Command runAtSpeed(DoubleSupplier speedSupplier) {
        return commandBuilder(getMethodInfo("supplier"))
            .onExecute(() -> setTargetSpeed(speedSupplier.getAsDouble()))
            .onEnd(this::stop);
    }

    /**
     * Runs the rollers at the specified {@code speed}.
     * @param speed The speed to run the rollers at. Speeds should be between 1.0 and -1.0.
     */
    private Command runAtSpeed(double speed) {
        return runAtSpeed(() -> speed).withName(getMethodInfo(String.valueOf(speed)));
    }

    /**
     * Runs the intake at the {@link GooseneckRollers#kIntakeSpeed kIntakeSpeed}.
     */
    public Command intake() {
        return runAtSpeed(Speeds.kIntake::getSpeed).withName(getMethodInfo());
    }

    /**
     * Runs the intake at the {@link GooseneckRollers#kScoreSpeed}.
     */
    public Command score() {
        return runAtSpeed(Speeds.kScore::getSpeed).withName(getMethodInfo());
    }

    /**
     * Indexes coral in the rollers.
     */
    public Command indexPiece() {
        return sequence(
            deadline(sequence(waitUntil(this::hasPiece), waitUntil(() -> !hasPiece())), intake()),
            runAtSpeed(Speeds.kIndexing::getSpeed).until(this::hasPiece)
        ).withName(getMethodInfo());
    }
}
