package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import org.team340.robot.Robot;
import org.team340.robot.subsystems.Elevator;
import org.team340.robot.subsystems.Elevator.ElevatorPosition;
import org.team340.robot.subsystems.GooseNeck;
import org.team340.robot.subsystems.Intake;
import org.team340.robot.subsystems.Lights;
import org.team340.robot.subsystems.Swerve;
import org.team340.robot.util.ReefSelection;

/**
 * The Routines class contains command compositions, such as sequences
 * or parallel command groups, that require multiple subsystems.
 */
@SuppressWarnings("unused")
@Logged(strategy = Strategy.OPT_IN)
public final class Routines {

    private final Robot robot;

    private final Elevator elevator;
    private final GooseNeck gooseNeck;
    private final Intake intake;
    private final Lights lights;
    private final Swerve swerve;

    private final ReefSelection selection;

    public Routines(Robot robot) {
        this.robot = robot;
        elevator = robot.elevator;
        gooseNeck = robot.gooseNeck;
        intake = robot.intake;
        lights = robot.lights;
        swerve = robot.swerve;
        selection = robot.selection;
    }

    /**
     * Runs the intake.
     */
    public Command intake() {
        return intake(() -> true);
    }

    /**
     * Runs the intake. A button supplier is specified for scheduling purposes,
     * allowing this command to be bound using {@link Trigger#onTrue()}. The
     * intake will continue running if a coral has been detected, but is not
     * yet in the goose neck.
     * @param button If the intake button is still pressed.
     */
    public Command intake(BooleanSupplier button) {
        return deadline(
            gooseNeck.intake(button, robot::safeForGoose),
            intake.intake(),
            elevator.goTo(ElevatorPosition.kIntake, robot::safeForGoose)
        ).withName("Routines.intake()");
    }

    /**
     * Runs the intake. A button supplier is specified for scheduling purposes,
     * allowing this command to be bound using {@link Trigger#onTrue()}. The
     * intake will continue running if a coral has been detected, but is not
     * yet in the goose neck. Will also slow down the drivetrain.
     * @param button If the intake button is still pressed.
     */
    public Command assistedIntake(BooleanSupplier button) {
        return deadline(
            intake(button),
            swerve.driveIntake(robot::driverX, robot::driverY, robot::driverAngular)
        ).withName("Routines.assistedIntake()");
    }

    /**
     * Barfs a coral out of the robot. Also clears
     * the goose neck's "has coral" state.
     */
    public Command barf() {
        return parallel(
            gooseNeck.barf(robot::safeForGoose),
            intake.barf(),
            elevator.goTo(ElevatorPosition.kBarf, robot::safeForGoose)
        ).withName("Routines.barf()");
    }

    /**
     * Swallows a coral into the robot. Also clears
     * the goose neck's "has coral" state.
     */
    public Command swallow() {
        return parallel(
            gooseNeck.swallow(robot::safeForGoose),
            intake.swallow(),
            elevator.goTo(ElevatorPosition.kSwallow, robot::safeForGoose)
        ).withName("Routines.swallow()");
    }

    /**
     * Scores a coral. Also allows the goose neck to goose around.
     * @param forceBeak Forces the goose beak to spit the coral, even if a pipe is not detected.
     */
    public Command score(boolean forceBeak) {
        return score(() -> forceBeak, () -> true).withName("Routines.score()");
    }

    /**
     * Scores a coral.
     * @param runManual A boolean supplier that when {@code true} will force the goose beak to spit, even if a pipe is not detected.
     * @param allowGoosing If the goose neck is allowed to goose around.
     */
    public Command score(BooleanSupplier runManual, BooleanSupplier allowGoosing) {
        return parallel(
            elevator.score(selection, robot::safeForGoose),
            gooseNeck.score(selection, runManual, allowGoosing, robot::safeForGoose).asProxy(),
            selection.whileScoring()
        ).withName("Routines.score()");
    }

    /**
     * Scores a coral, with driver assists. The drivetrain will be "pushed" to
     * center itself on a reef pipe, and the robot will also face the reef.
     * @param runManual A boolean supplier that when {@code true} will force the goose beak to spit, even if a pipe is not detected.
     * @param allowGoosing If the goose neck is allowed to goose around.
     */
    public Command assistedScore(BooleanSupplier runManual, BooleanSupplier allowGoosing) {
        return parallel(
            score(runManual, allowGoosing),
            swerve.driveReef(robot::driverX, robot::driverY, robot::driverAngular, selection::isLeft)
        ).withName("Routines.assistedScore()");
    }
}
