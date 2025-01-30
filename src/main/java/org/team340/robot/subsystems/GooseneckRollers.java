package org.team340.robot.subsystems;

import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.DoubleSupplier;

import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.robot.Constants.RobotMap;

@Logged
public class GooseneckRollers extends GRRSubsystem {

    private static final TunableDouble kIntakeSpeed = Tunable.doubleValue("GooseneckRollers/kIntakeSpeed", 0.0);
    private static final TunableDouble kScoreSpeed = Tunable.doubleValue("GooseneckRollers/kScoreSpeed",0.0);
    private static final TunableDouble kIndexingSpeed = Tunable.doubleValue("GooseneckRollers/kIndexingSpeed", 0.0);

    private final TalonFXS rollerMotor;
    private final CANdi beamBreak;

    public GooseneckRollers() {
        rollerMotor = new TalonFXS(RobotMap.kGooseneckRollersMotor);
        beamBreak = new CANdi(RobotMap.kGooseneckRCANdi);
    }

    private void applySpeed(double speed) {
        rollerMotor.set(speed);
    }

    private void stop() {
        rollerMotor.stopMotor();
    }

    public boolean hasPiece() {
        return beamBreak.getS1Closed().getValue();
    }

    public boolean notHasPiece() {
        return !hasPiece();
    }

    public Command runAtSpeed(DoubleSupplier speed) {
        return commandBuilder("GooseneckRollers.runAtSpeed(" + speed + ")")
            .onExecute(() -> applySpeed(speed.getAsDouble()))
            .onEnd(this::stop);
    }

    public Command runAtSpeed(double speed) {
        return runAtSpeed(() -> speed);
    }

    public Command intake() {
        return runAtSpeed(kIntakeSpeed::value);
    }

    public Command score() {
        return runAtSpeed(kScoreSpeed::value);
    }

    public Command indexPiece() {
        return sequence(
            deadline(
                sequence(waitUntil(this::hasPiece), waitUntil(this::notHasPiece)),
                intake()
            ),
            runAtSpeed(kIndexingSpeed::value).until(this::hasPiece)
        );
    }
}
