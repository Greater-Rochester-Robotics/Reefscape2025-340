package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableBoolean;
import org.team340.lib.util.Mutable;
import org.team340.robot.Robot;
import org.team340.robot.subsystems.Climber;
import org.team340.robot.subsystems.Elevator;
import org.team340.robot.subsystems.Elevator.ElevatorPosition;
import org.team340.robot.subsystems.GooseNeck;
import org.team340.robot.subsystems.Intake;
import org.team340.robot.subsystems.Swerve;
import org.team340.robot.util.ReefSelection;

/**
 * The Routines class contains command compositions, such as sequences
 * or parallel command groups, that require multiple subsystems.
 */
@SuppressWarnings("unused")
public final class Routines {

    private static final TunableTable tunables = Tunables.getTable("routines");

    private static final TunableBoolean autoDrive = tunables.value("autoDrive", true);

    private final Robot robot;

    private final Climber climber;
    private final Elevator elevator;
    private final GooseNeck gooseNeck;
    private final Intake intake;
    // private final Lights lights;
    private final Swerve swerve;

    private final ReefSelection selection;

    public Routines(Robot robot) {
        this.robot = robot;
        climber = robot.climber;
        elevator = robot.elevator;
        gooseNeck = robot.gooseNeck;
        intake = robot.intake;
        // lights = robot.lights;
        swerve = robot.swerve;
        selection = robot.selection;
    }

    /**
     * Stows the elevator and goose neck.
     */
    public Command stow() {
        return parallel(
            elevator.goTo(ElevatorPosition.DOWN, robot::safeForGoose),
            gooseNeck.stow(robot::safeForGoose)
        ).withName("Routines.stow()");
    }

    /**
     * Intakes and seats a coral.
     */
    public Command intake() {
        return intake(() -> true);
    }

    /**
     * Intakes and seats a coral. A button supplier is specified for scheduling purposes,
     * allowing this command to be bound using {@link Trigger#onTrue()}. The
     * intake will continue running if a coral has been detected, but is not
     * yet in the goose neck.
     * @param button If the intake button is still pressed.
     */
    public Command intake(BooleanSupplier button) {
        Debouncer debounce = new Debouncer(1.0);
        Timer chokeTimer = new Timer();

        BooleanSupplier chokingGoose = () -> {
            if (debounce.calculate(gooseNeck.beamBroken())) chokeTimer.start();

            if (chokeTimer.isRunning() && !chokeTimer.hasElapsed(0.45)) {
                return true;
            } else {
                chokeTimer.stop();
                chokeTimer.reset();
                return false;
            }
        };

        return sequence(
            deadline(
                waitUntil(elevator::safeForIntake),
                gooseNeck.stow(robot::safeForGoose),
                elevator.goTo(ElevatorPosition.INTAKE, robot::safeForGoose),
                intake.swallow()
            ),
            deadline(
                gooseNeck.intake(button, () -> chokingGoose.getAsBoolean() || intake.unjamming(), robot::safeForGoose),
                elevator.goTo(ElevatorPosition.INTAKE, robot::safeForGoose),
                intake.intake(chokingGoose)
            )
        ).withName("Routines.intake()");
    }

    /**
     * Barfs a coral out of the robot. Also clears
     * the goose neck's "has coral" state.
     */
    public Command barf() {
        return parallel(
            gooseNeck.barf(robot::safeForGoose),
            intake.barf(),
            elevator.goTo(ElevatorPosition.BARF, swerve::wildlifeConservationProgram) // Ignore beam break in safety check
        ).withName("Routines.barf()");
    }

    /**
     * Swallows a coral into the robot. Also clears
     * the goose neck's "has coral" state.
     */
    public Command swallow() {
        return parallel(
            intake.swallow(),
            gooseNeck
                .swallow(robot::safeForGoose)
                .beforeStarting(gooseNeck.stow(robot::safeForGoose).withTimeout(0.05)),
            elevator.goTo(ElevatorPosition.SWALLOW, swerve::wildlifeConservationProgram) // Ignore beam break in safety check
        ).withName("Routines.swallow()");
    }

    /**
     * Scores a coral.
     * @param runManual A boolean supplier that when {@code true} will force the
     *                  goose beak to spit, even if a pipe is not detected.
     * @param allowGoosing If the goose neck is allowed to goose around.
     */
    public Command score(BooleanSupplier runManual, BooleanSupplier allowGoosing) {
        Mutable<Boolean> hadCoral = new Mutable<>(false);

        return sequence(
            deadline(
                waitUntil(swerve::happyGoose),
                elevator.goTo(ElevatorPosition.DOWN, robot::safeForGoose),
                gooseNeck.stow(robot::safeForGoose)
            ),
            deadline(
                waitUntil(
                    () -> hadCoral.value && gooseNeck.noCoral() && robot.safeForGoose() && allowGoosing.getAsBoolean()
                ),
                elevator.score(
                    selection,
                    () ->
                        hadCoral.value &&
                        gooseNeck.beamBroken() &&
                        !runManual.getAsBoolean() &&
                        swerve.getVelocity() > 0.25,
                    robot::safeForGoose
                ),
                gooseNeck.score(
                    selection,
                    runManual,
                    () -> allowGoosing.getAsBoolean() && swerve.goosingTime(),
                    robot::safeForGoose
                )
            ),
            parallel(elevator.goTo(ElevatorPosition.DOWN, robot::safeForGoose), gooseNeck.stow(robot::safeForGoose))
        )
            .alongWith(selection.whileScoring())
            .beforeStarting(() -> hadCoral.value = gooseNeck.hasCoral())
            .withName("Routines.score()");
    }

    /**
     * Scores a coral, with driver assists. Intended to be used during teleop control.
     * @param runManual A boolean supplier that when {@code true} will force the
     *                  goose beak to spit, even if a pipe is not detected.
     * @param allowGoosing If the goose neck is allowed to goose around.
     */
    public Command assistedScore(BooleanSupplier runManual, BooleanSupplier allowGoosing) {
        return parallel(
            score(runManual, allowGoosing),
            either(
                swerve.driveReef(robot::driverX, robot::driverY, robot::driverAngular, selection::isLeft),
                sequence(
                    swerve.apfDrive(selection::isLeft, robot::readyToScore, selection::isL4).until(gooseNeck::noCoral),
                    swerve.drive(robot::driverX, robot::driverY, robot::driverAngular)
                ),
                () -> !autoDrive.get() || gooseNeck.noCoral()
            )
        ).withName("Routines.assistedScore()");
    }

    /**
     * Kills the goose :( and elevator
     * For emergencies
     */
    public Command killTheGoose() {
        // return parallel(idle(elevator, gooseNeck), lights.top.gooseAssassination())
        return parallel(idle(elevator, gooseNeck))
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
            .withName("Routines.killTheGoose()");
    }
}
