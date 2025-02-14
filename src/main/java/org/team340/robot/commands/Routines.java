package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static org.team340.lib.util.command.GRRUtilities.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.robot.Robot;
import org.team340.robot.subsystems.Elevator;
import org.team340.robot.subsystems.GooseBeak;
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
    private final GooseBeak gooseBeak;
    private final GooseNeck gooseNeck;
    private final Intake intake;
    private final Swerve swerve;

    public Routines(Robot robot) {
        swerve = robot.swerve;
        elevator = robot.elevator;
        gooseBeak = robot.gooseBeak;
        gooseNeck = robot.gooseNeck;
        intake = robot.intake;
    }

    /**
     * Intakes a piece of coral from the intake to the goose beak.
     */
    public Command intake() {
        return sequence(
            parallel(elevator.goTo(Elevator.Position.kLoad), gooseNeck.goToPosition(GooseNeck.Position.kIn)),
            parallel(intake.intake(), gooseBeak.intake()).until(gooseBeak::hasCoral)
        ).withName(getMethodInfo());
    }
}
