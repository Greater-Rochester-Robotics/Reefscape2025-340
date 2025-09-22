package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static org.team340.robot.util.Field.ReefLocation.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Function;
import java.util.function.Supplier;
import org.team340.lib.math.geometry.ExtPose;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.util.Alliance;
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
public final class Autos {

    private static final TunableTable tunables = Tunables.getNested("autos");

    private static final TunableDouble intakeDecel = tunables.value("intakeDecel", 6.0);
    private static final TunableDouble intakeRotDelay = tunables.value("intakeRotDelay", 0.6);
    private static final TunableDouble avoidDecel = tunables.value("avoidDecel", 8.9);
    private static final TunableDouble avoidTol = tunables.value("avoidTol", 0.25);

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
        chooser.add("test2", test2());
        // chooser.add("test12 left", test12(true));
        // chooser.add("test12 right", test12(false));
    }

    /**
     * Returns {@code true} when the default auto is selected.
     */
    public boolean defaultSelected() {
        return chooser.defaultSelected().getAsBoolean();
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
        ExtPose direction = sideLoad ? (start.back ? Field.loadBackwards : Field.loadForwards) : Field.loadStraight;

        Supplier<Pose2d> station = () -> direction.get(left);
        Timer timer = new Timer();

        return swerve
            .apfDrive(
                () -> {
                    Pose2d pose = station.get();
                    if (!timer.hasElapsed(intakeRotDelay.get())) {
                        return new Pose2d(
                            pose.getX(),
                            pose.getY(),
                            Alliance.isBlue() ? start.side : start.side.rotateBy(Rotation2d.k180deg)
                        );
                    } else {
                        return pose;
                    }
                },
                intakeDecel::get
            )
            .until(() -> intake.coralDetected() || gooseNeck.hasCoral())
            .beforeStarting(timer::restart);
    }

    private Command avoid(boolean left) {
        return swerve.apfDrive(() -> Field.avoid.get(left), avoidDecel::get, avoidTol::get);
    }

    // ********** Sim / Testing **********

    private Command test2() {
        Function<ReefLocation, Command> goReef = reefLocation ->
            swerve
                .apfDrive(reefLocation, robot::readyToScore, selection::isL4)
                .withDeadline(sequence(waitUntil(() -> !swerve.wildlifeConservationProgram()), waitSeconds(0.75)));

        Supplier<Command> goIntake = () -> swerve.apfDrive(Field.loadStraight, intakeDecel::get, avoidTol::get);

        return sequence(
            swerve.resetPose(new ExtPose(7.0, 6.5, Rotation2d.kZero)),
            goReef.apply(ReefLocation.A),
            avoid(true),
            goReef.apply(ReefLocation.E),
            goIntake.get(),
            goReef.apply(ReefLocation.B),
            avoid(false),
            goReef.apply(ReefLocation.K),
            goIntake.get(),
            goReef.apply(ReefLocation.B),
            avoid(true),
            goReef.apply(ReefLocation.J),
            avoid(true),
            goReef.apply(ReefLocation.K),
            avoid(false),
            goIntake.get(),
            goReef.apply(ReefLocation.L),
            avoid(false)
        );
    }

    private Command test12(boolean left) {
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
}
