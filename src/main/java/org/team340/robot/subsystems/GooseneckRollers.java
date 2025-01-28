package org.team340.robot.subsystems;

import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.DoubleSupplier;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.robot.Constants.RobotMap;

public class GooseneckRollers extends GRRSubsystem {

    final double kIntakeSpeed = 0.0;
    final double kScoreSpeed = 0.0;
    final double kIndexingSpeed = 0.0;

    final TalonFXS rollerMotor;
    final CANdi beamBreak;

    public GooseneckRollers() {
        rollerMotor = new TalonFXS(RobotMap.kGooseneckRollersMotor);
        beamBreak = new CANdi(RobotMap.kGooseneckRollersBeamBreak);
    }

    public void applySpeed(double speed) {
        rollerMotor.set(speed);
    }

    public void stop() {
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
        return runAtSpeed(kIntakeSpeed);
    }

    public Command score() {
        return runAtSpeed(kScoreSpeed);
    }

    public Command indexPiece() {
        return sequence(
            deadline(
                sequence(waitUntil(this::hasPiece), waitUntil(this::notHasPiece)),
                intake()
            ),
            runAtSpeed(kIndexingSpeed).until(this::hasPiece)
        );
    }
}
