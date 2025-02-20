package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.team340.robot.Robot;
import org.team340.robot.subsystems.Elevator;
import org.team340.robot.subsystems.Elevator.ElevatorPosition;
import org.team340.robot.subsystems.GooseNeck;
import org.team340.robot.subsystems.Intake;
import org.team340.robot.subsystems.Lights;
import org.team340.robot.subsystems.Swerve;

/**
 * The Routines class contains command compositions, such as sequences
 * or parallel command groups, that require multiple subsystems.
 */
@SuppressWarnings("unused")
@Logged(strategy = Strategy.OPT_IN)
public final class Routines {

    private final Elevator elevator;
    private final GooseNeck gooseNeck;
    private final Intake intake;
    private final Lights lights;
    private final Swerve swerve;

    private final BooleanSupplier safeForGoose;

    public Routines(Robot robot) {
        elevator = robot.elevator;
        gooseNeck = robot.gooseNeck;
        intake = robot.intake;
        lights = robot.lights;
        swerve = robot.swerve;

        safeForGoose = robot::safeForGoose;
    }

    public Command intake(BooleanSupplier button) {
        return deadline(
            gooseNeck.intake(button, safeForGoose),
            intake.intake().asProxy(),
            elevator.goTo(ElevatorPosition.kIntake, safeForGoose)
        ).withName("Routines.intake()");
    }

    public Command barf() {
        return parallel(
            gooseNeck.barf(safeForGoose),
            intake.barf(),
            elevator.goTo(ElevatorPosition.kBarf, safeForGoose)
        ).withName("Routines.barf()");
    }

    public Command swallow() {
        return parallel(
            gooseNeck.swallow(safeForGoose),
            intake.swallow(),
            elevator.goTo(ElevatorPosition.kSwallow, safeForGoose)
        ).withName("Routines.swallow()");
    }

    public Command score(
        DoubleSupplier x,
        DoubleSupplier y,
        Supplier<ElevatorPosition> position,
        BooleanSupplier runManual,
        boolean left,
        boolean allowMovement
    ) {
        return parallel(
            swerve.driveReef(x, y),
            elevator.goTo(position, safeForGoose),
            gooseNeck
                .score(runManual, () -> position.get().equals(ElevatorPosition.kL1), safeForGoose, left, allowMovement)
                .asProxy()
        ).withName("Routines.score()");
    }
}
