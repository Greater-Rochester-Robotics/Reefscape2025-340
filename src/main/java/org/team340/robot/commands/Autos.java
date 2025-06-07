package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static org.team340.robot.util.Field.ReefLocation.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.team340.lib.util.Alliance;
import org.team340.lib.util.FieldFlip;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.lib.util.command.AutoChooser;
import org.team340.robot.Robot;
import org.team340.robot.subsystems.Elevator;
import org.team340.robot.subsystems.GooseNeck;
import org.team340.robot.subsystems.Intake;
import org.team340.robot.subsystems.Lights;
import org.team340.robot.subsystems.Swerve;
import org.team340.robot.util.Field;
import org.team340.robot.util.Field.ReefLocation;
import org.team340.robot.util.ReefSelection;

/**
 * The Autos class declares autonomous modes, and adds them
 * to the dashboard to be selected by the drive team.
 */
@SuppressWarnings("unused")
@Logged(strategy = Strategy.OPT_IN)
public final class Autos {

    private static final TunableDouble INTAKE_SLOWDOWN = Tunable.value("autos/intakeSlowdown", 8.7);
    private static final TunableDouble INTAKE_ROT_DELAY = Tunable.value("autos/intakeRotDelay", 0.6);
    private static final TunableDouble AVOID_SLOWDOWN = Tunable.value("autos/avoidSlowdown", 8.9);
    private static final TunableDouble AVOID_TOLERANCE = Tunable.value("autos/avoidTolerance", 0.25);

    private final Robot robot;

    private final Elevator elevator;
    private final GooseNeck gooseNeck;
    private final Intake intake;
    private final Lights lights;
    private final Swerve swerve;

    private final Routines routines;
    private final ReefSelection selection;

    private final AutoChooser chooser;

    public Autos(Robot robot) {
        this.robot = robot;

        elevator = robot.elevator;
        gooseNeck = robot.gooseNeck;
        intake = robot.intake;
        lights = robot.lights;
        swerve = robot.swerve;

        selection = robot.selection;
        routines = robot.routines;

        // Create the auto chooser
        chooser = new AutoChooser();

        // Add autonomous modes to the dashboard
        chooser.add("For Piece Left", forPiece(true));
        chooser.add("For Piece Right", forPiece(false));
        chooser.add("Sneaky Two Left", sneakyTwo(true));
        chooser.add("Sneaky Two Right", sneakyTwo(false));
        chooser.add("Stinky One Left", stinkyOne(true));
        chooser.add("Stinky One Right", stinkyOne(false));
        chooser.add("One Billion Points Left", oneBillionPoints(true));
        chooser.add("One Billion Points Right", oneBillionPoints(false));
        // chooser.add("Test 12 Left", test12(true));
        // chooser.add("Test 12 Right", test12(false));

        // Chooser bindings
        chooser.newSelection().onTrue(lights.top.scored().andThen(lights.disabled()));
    }

    private Command forPiece(boolean left) {
        return parallel(
            sequence(
                parallel(setupL4(), pickupCycle(left ? I : F, true, left)),
                pickupCycle(left ? K : D, true, left),
                pickupCycle(left ? L : C, true, left),
                pickupCycle(left ? A : B, true, left),
                pickupCycle(left ? B : A, true, left)
            ),
            sequence(
                routines.score(() -> false, () -> true).until(() -> gooseNeck.noCoral() && robot.safeForGoose()),
                routines.intake()
            ).repeatedly()
        );
    }

    private Command oneBillionPoints(boolean left) {
        return parallel(
            sequence(
                parallel(setupL4(), pickupCycle(left ? I : F, false, left)),
                pickupCycle(left ? K : D, false, left),
                pickupCycle(left ? L : C, false, left),
                selection.selectLevel(3),
                pickupCycle(left ? K : D, false, left),
                pickupCycle(left ? L : C, false, left),
                selection.selectLevel(4),
                pickupCycle(left ? A : B, false, left)
            ),
            sequence(
                routines.score(() -> false, () -> true).until(() -> gooseNeck.noCoral() && robot.safeForGoose()),
                routines.intake()
            ).repeatedly()
        );
    }

    private Command sneakyTwo(boolean left) {
        return parallel(
            sequence(
                parallel(setupL4(), score(left ? G : H, left)),
                avoid(left),
                pickup(G, true, left),
                avoid(left),
                score(left ? H : G, left),
                avoid(left),
                swerve.stop(false)
            ),
            sequence(
                routines.score(() -> false, () -> true).until(() -> gooseNeck.noCoral() && robot.safeForGoose()),
                routines.intake()
            ).repeatedly()
        );
    }

    private Command stinkyOne(boolean left) {
        return sequence(
            setupL4(),
            deadline(waitSeconds(2.5), swerve.stop(false), routines.stow()),
            parallel(
                sequence(score(left ? G : H, left), avoid(left), swerve.stop(false)),
                sequence(
                    routines.score(() -> false, () -> true).until(() -> gooseNeck.noCoral() && robot.safeForGoose()),
                    routines.stow()
                )
            )
        );
    }

    public Command test12(boolean left) {
        return parallel(
            sequence(
                pickupCycle(left ? I : F, true, left),
                pickupCycle(left ? C : L, true, left),
                pickupCycle(left ? G : H, true, left),
                pickupCycle(left ? L : C, true, left),
                pickupCycle(left ? A : B, true, left),
                pickupCycle(left ? D : K, true, left),
                pickupCycle(left ? B : A, true, left),
                pickupCycle(left ? E : J, true, left),
                pickupCycle(left ? F : I, true, left),
                pickupCycle(left ? H : G, true, left),
                pickupCycle(left ? J : E, true, left),
                pickupCycle(left ? K : D, true, left)
            ),
            sequence(
                routines.score(() -> false, () -> true).until(() -> gooseNeck.noCoral() && robot.safeForGoose()),
                routines.intake()
            ).repeatedly()
        ).beforeStarting(parallel(selection.selectLevel(3), gooseNeck.setHasCoral(true)));
    }

    private Command setupL4() {
        return parallel(selection.selectLevel(4), gooseNeck.setHasCoral(true));
    }

    private Command pickupCycle(ReefLocation reefLocation, boolean sideLoad, boolean left) {
        return sequence(score(reefLocation, left), pickup(reefLocation, sideLoad, left));
    }

    private Command score(ReefLocation reefLocation, boolean left) {
        return deadline(
            sequence(
                waitUntil(gooseNeck::hasCoral),
                waitUntil(() -> !swerve.wildlifeConservationProgram() && gooseNeck.noCoral())
            ),
            swerve.apfDrive(reefLocation, robot::readyToScore, selection::isL4)
        );
    }

    private Command pickup(ReefLocation start, boolean sideLoad, boolean left) {
        Pose2d direction = sideLoad
            ? (start.back ? Field.STATION_BACKWARDS : Field.STATION_FORWARDS)
            : Field.STATION_STRAIGHT;
        Supplier<Pose2d> station = FieldFlip.allianceDiagonal(left ? FieldFlip.overWidth(direction) : direction);
        Timer timer = new Timer();

        return swerve
            .apfDrive(
                () -> {
                    Pose2d pose = station.get();
                    if (!timer.hasElapsed(INTAKE_ROT_DELAY.get())) {
                        return new Pose2d(
                            pose.getX(),
                            pose.getY(),
                            Alliance.isBlue() ? start.side : start.side.rotateBy(Rotation2d.kPi)
                        );
                    } else {
                        return pose;
                    }
                },
                INTAKE_SLOWDOWN::get
            )
            .until(() -> intake.coralDetected() || gooseNeck.hasCoral())
            .beforeStarting(timer::restart);
    }

    private Command avoid(boolean left) {
        return swerve.apfDrive(
            FieldFlip.allianceDiagonal(left ? Field.AVOID_LEFT : Field.AVOID_RIGHT),
            AVOID_SLOWDOWN::get,
            AVOID_TOLERANCE::get
        );
    }
}
