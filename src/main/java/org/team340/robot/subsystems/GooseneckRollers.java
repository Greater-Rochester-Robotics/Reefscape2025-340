package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.robot.Constants.RobotMap;

@Logged
public class GooseneckRollers extends GRRSubsystem {

    private static final TunableDouble kIntakeSpeed = Tunable.doubleValue("GooseneckRollers/kIntakeSpeed", 0.0);
    private static final TunableDouble kScoreSpeed = Tunable.doubleValue("GooseneckRollers/kScoreSpeed", 0.0);
    private static final TunableDouble kIndexingSpeed = Tunable.doubleValue("GooseneckRollers/kIndexingSpeed", 0.0);

    private final TalonFXS rollerMotor;
    private final CANdi beamBreak;

    public GooseneckRollers() {
        rollerMotor = new TalonFXS(RobotMap.kGooseneckRollersMotor);
        beamBreak = new CANdi(RobotMap.kGooseneckRCANdi);
    }

    /**
     * Sets the target speed of the rollers.
     * @param speed The target speed. Speeds should be between 1.0 and -1.0.
     */
    private void setTargetSpeed(double speed) {
        rollerMotor.set(speed);
    }

    /**
     * Stops the roller motor.
     */
    private void stop() {
        rollerMotor.stopMotor();
    }

    /**
     * Checks if the beam break detects an object.
     * @return True if the beam break detects an object, false otherwise.
     */
    public boolean hasPiece() {
        return beamBreak.getS1Closed().getValue();
    }

    /**
     * Checks if the beam break does not detect an object.
     * @return True if the beam break does not detect an object, false otherwise.
     */
    public boolean notHasPiece() {
        return !hasPiece();
    }

    /**
     * Runs the rollers at the speed supplied by {@code speedSupplier}.
     * @param speedSupplier Supplies the speed the rollers are run at. Speeds should be between 1.0 and -1.0
     */
    public Command runAtSpeed(DoubleSupplier speedSupplier) {
        return commandBuilder("GooseneckRollers.runAtSpeed(supplier)")
            .onExecute(() -> setTargetSpeed(speedSupplier.getAsDouble()))
            .onEnd(this::stop);
    }

    /**
     * Runs the rollers at the specified {@code speed}.
     * @param speed The speed to run the rollers at. Speeds should be between 1.0 and -1.0.
     */
    public Command runAtSpeed(double speed) {
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
            deadline(sequence(waitUntil(this::hasPiece), waitUntil(this::notHasPiece)), intake()),
            runAtSpeed(kIndexingSpeed::value).until(this::hasPiece)
        ).withName("GooseneckRollers.indexPiece()");
    }
}
