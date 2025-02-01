package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
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

    private static final TunableDouble kIntakeSpeed = Tunable.doubleValue("GooseneckRollers/kIntakeSpeed", 0.0);
    private static final TunableDouble kScoreSpeed = Tunable.doubleValue("GooseneckRollers/kScoreSpeed", 0.0);
    private static final TunableDouble kIndexingSpeed = Tunable.doubleValue("GooseneckRollers/kIndexingSpeed", 0.0);

    private final TalonFXS rollerMotor;
    private final CANdi beamBreak;

    public GooseBeak() {
        rollerMotor = new TalonFXS(RobotMap.kGooseBeakMotor, RobotMap.kUpperCANBus);

        TalonFXSConfiguration rollerConfig = new TalonFXSConfiguration();

        rollerConfig.CurrentLimits.StatorCurrentLimit = 0.0;
        rollerConfig.CurrentLimits.SupplyCurrentLimit = 0.0;

        PhoenixUtil.run("Apply the TalonFXSConfiguration to the rollerMotor", rollerMotor, () ->
            rollerMotor.getConfigurator().apply(rollerConfig)
        );

        beamBreak = new CANdi(RobotMap.kGooseBeakCANdi, RobotMap.kUpperCANBus);
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
        return (
            RobotMap.kGooseBeakCANdiPort == ForwardLimitSourceValue.RemoteCANdiS1
                ? beamBreak.getS1Closed()
                : beamBreak.getS2Closed()
        ).getValue();
    }

    // *************** Commands ***************

    /**
     * Runs the rollers at the speed supplied by {@code speedSupplier}.
     * @param speedSupplier Supplies the speed the rollers are run at. Speeds should be between 1.0 and -1.0
     */
    private Command runAtSpeed(DoubleSupplier speedSupplier) {
        return commandBuilder("GooseneckRollers.runAtSpeed(supplier)")
            .onExecute(() -> setTargetSpeed(speedSupplier.getAsDouble()))
            .onEnd(this::stop);
    }

    /**
     * Runs the rollers at the specified {@code speed}.
     * @param speed The speed to run the rollers at. Speeds should be between 1.0 and -1.0.
     */
    private Command runAtSpeed(double speed) {
        return runAtSpeed(() -> speed).withName("GooseneckRollers.runAtSpeed(" + speed + ")");
    }

    /**
     * Runs the intake at the {@link GooseneckRollers#kIntakeSpeed kIntakeSpeed}.
     */
    public Command intake() {
        return runAtSpeed(kIntakeSpeed::value).withName("GooseneckRollers.intake()");
    }

    /**
     * Runs the intake at the {@link GooseneckRollers#kScoreSpeed}.
     */
    public Command score() {
        return runAtSpeed(kScoreSpeed::value).withName("GooseneckRollers.score()");
    }

    /**
     * Indexes coral in the rollers.
     */
    public Command indexPiece() {
        return sequence(
            deadline(sequence(waitUntil(this::hasPiece), waitUntil(() -> !hasPiece())), intake()),
            runAtSpeed(kIndexingSpeed::value).until(this::hasPiece)
        ).withName("GooseneckRollers.indexPiece()");
    }
}
