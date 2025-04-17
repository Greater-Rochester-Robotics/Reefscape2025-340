package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableBoolean;
import org.team340.robot.Robot;
import org.team340.robot.subsystems.Climber;
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

    private static final TunableBoolean kAutoDrive = Tunable.booleanValue("routines/kAutoDrive", true);

    private final Robot robot;

    private final Climber climber;
    private final Elevator elevator;
    private final GooseNeck gooseNeck;
    private final Intake intake;
    private final Lights lights;
    private final Swerve swerve;

    private final ReefSelection selection;

    public Routines(Robot robot) {
        this.robot = robot;
        climber = robot.climber;
        elevator = robot.elevator;
        gooseNeck = robot.gooseNeck;
        intake = robot.intake;
        lights = robot.lights;
        swerve = robot.swerve;
        selection = robot.selection;
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
                elevator.goTo(ElevatorPosition.kIntake, robot::safeForGoose),
                intake.swallow()
            ),
            deadline(
                gooseNeck.intake(button, () -> chokingGoose.getAsBoolean() || intake.unjamming(), robot::safeForGoose),
                elevator.goTo(ElevatorPosition.kIntake, robot::safeForGoose),
                intake.intake(chokingGoose).beforeStarting(waitSeconds(0.1))
            )
        ).withName("Routines.intake()");
    }

    /**
     * Intakes from the coral station via the baby bird method.
     * @param button If the intake button is still pressed.
     */
    public Command babyBird(BooleanSupplier button) {
        return deadline(
            gooseNeck.babyBird(button, robot::safeForGoose),
            elevator.goTo(ElevatorPosition.kBabyBird, robot::safeForGoose)
        ).withName("Routines.babyBird()");
    }

    /**
     * Barfs a coral out of the robot. Also clears
     * the goose neck's "has coral" state.
     */
    public Command barf() {
        return parallel(
            gooseNeck.barf(robot::safeForGoose),
            intake.barf(),
            elevator.goTo(ElevatorPosition.kBarf, swerve::wildlifeConservationProgram) // Ignore beam break in safety check
        ).withName("Routines.barf()");
    }

    /**
     * Swallows a coral into the robot. Also clears
     * the goose neck's "has coral" state.
     */
    public Command swallow() {
        return parallel(
            intake.swallow(),
            gooseNeck.swallow(robot::safeForGoose).beforeStarting(gooseNeck.stow(robot::safeForGoose).withTimeout(0.1)),
            elevator.goTo(ElevatorPosition.kSwallow, swerve::wildlifeConservationProgram) // Ignore beam break in safety check
        ).withName("Routines.swallow()");
    }

    /**
     * Scores a coral.
     * @param runManual A boolean supplier that when {@code true} will force the
     *                  goose beak to spit, even if a pipe is not detected.
     * @param allowGoosing If the goose neck is allowed to goose around.
     */
    public Command score(BooleanSupplier runManual, BooleanSupplier allowGoosing) {
        return sequence(
            deadline(
                waitUntil(swerve::happyGoose),
                elevator.goTo(ElevatorPosition.kDown, robot::safeForGoose),
                gooseNeck.stow(robot::safeForGoose)
            ),
            deadline(
                either(
                    waitUntil(() -> gooseNeck.noCoral() && robot.safeForGoose() && allowGoosing.getAsBoolean()),
                    idle(),
                    gooseNeck::hasCoral
                ),
                elevator.score(
                    selection,
                    () -> gooseNeck.beamBroken() && !runManual.getAsBoolean() && swerve.getVelocity() > 0.5,
                    robot::safeForGoose
                ),
                gooseNeck.score(
                    selection,
                    runManual,
                    () -> allowGoosing.getAsBoolean() && swerve.goosingTime(),
                    robot::safeForGoose
                )
            ),
            parallel(elevator.goTo(ElevatorPosition.kDown, robot::safeForGoose), gooseNeck.stow(robot::safeForGoose))
        )
            .alongWith(selection.whileScoring())
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
                    swerve
                        .repulsorDrive(selection::isLeft, robot::readyToScore, selection::isL4)
                        .until(gooseNeck::noCoral),
                    swerve.drive(robot::driverX, robot::driverY, robot::driverAngular)
                ),
                () -> !kAutoDrive.value() || gooseNeck.noCoral()
            )
        ).withName("Routines.assistedScore()");
    }

    /**
     * Kills the goose :( and elevator
     * For emergencies
     */
    public Command killTheGoose() {
        return parallel(idle(elevator, gooseNeck), lights.top.gooseAssassination())
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
            .withName("Routines.killTheGoose()");
    }
}
