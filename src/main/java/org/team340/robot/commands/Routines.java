package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.team340.robot.Robot;
import org.team340.robot.subsystems.Elevator;
import org.team340.robot.subsystems.Elevator.ElevatorPosition;
import org.team340.robot.subsystems.GooseNeck;
import org.team340.robot.subsystems.Intake;
import org.team340.robot.subsystems.Swerve;

/**
 * The Routines class contains command compositions, such as sequences
 * or parallel command groups, that require multiple subsystems.
 */
@Logged(strategy = Strategy.OPT_IN)
public final class Routines {

    private final Elevator elevator;
    private final GooseNeck gooseNeck;
    private final Intake intake;
    private final Swerve swerve;

    public Routines(Robot robot) {
        elevator = robot.elevator;
        gooseNeck = robot.gooseNeck;
        intake = robot.intake;
        swerve = robot.swerve;
    }

    public Command intake(BooleanSupplier button) {
        return deadline(
            gooseNeck.intake(button, swerve::safeForGoose),
            intake.intake(),
            elevator.goTo(ElevatorPosition.kIntake, swerve::safeForGoose)
        ).withName("Routines.intake()");
    }

    public Command scoreForward(
        Supplier<ElevatorPosition> position,
        BooleanSupplier runManual,
        boolean left,
        boolean allowMovement
    ) {
        return parallel(
            elevator.goTo(position, swerve::safeForGoose),
            gooseNeck.score(
                runManual,
                () -> position.get().equals(ElevatorPosition.kL1),
                swerve::safeForGoose,
                left,
                allowMovement
            )
        ).withName("Routines.scoreForward()");
    }
}
