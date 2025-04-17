package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static org.team340.robot.Constants.FieldConstants.ReefLocation.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team340.lib.util.Alliance;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.robot.Constants.FieldConstants;
import org.team340.robot.Constants.FieldConstants.ReefLocation;
import org.team340.robot.Robot;
import org.team340.robot.subsystems.Elevator;
import org.team340.robot.subsystems.GooseNeck;
import org.team340.robot.subsystems.Intake;
import org.team340.robot.subsystems.Lights;
import org.team340.robot.subsystems.Swerve;
import org.team340.robot.util.ReefSelection;

/**
 * The Autos class declares autonomous modes, and adds them
 * to the dashboard to be selected by the drive team.
 */
@SuppressWarnings("unused")
@Logged(strategy = Strategy.OPT_IN)
public final class Autos {

    private static final TunableDouble kIntakeSlowdown = Tunable.doubleValue("autos/kIntakeSlowdown", 0.68);
    private static final TunableDouble kIntakeRotDelay = Tunable.doubleValue("autos/kIntakeRotDelay", 0.6);
    private static final TunableDouble kAvoidSlowdown = Tunable.doubleValue("autos/kAvoidSlowdown", 0.65);
    private static final TunableDouble kAvoidTolerance = Tunable.doubleValue("autos/kAvoidTolerance", 0.25);

    private final Robot robot;

    private final Elevator elevator;
    private final GooseNeck gooseNeck;
    private final Intake intake;
    private final Lights lights;
    private final Swerve swerve;

    private final Routines routines;
    private final ReefSelection selection;

    private final AutoFactory factory;
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

        // Create the auto factory
        factory = new AutoFactory(swerve::getPose, swerve::resetPose, swerve::followTrajectory, true, swerve);
        chooser = new AutoChooser();

        // Add autonomous modes to the dashboard
        chooser.addCmd("For Piece Left", () -> forPiece(true));
        chooser.addCmd("For Piece Right", () -> forPiece(false));
        chooser.addRoutine("Ol' Reliable Left (x3.5 L4)", () -> olReliable(true));
        chooser.addRoutine("Ol' Reliable Right (x3.5 L4)", () -> olReliable(false));
        chooser.addCmd("Sneaky Two Left", () -> sneakyTwo(true));
        chooser.addCmd("Sneaky Two Right", () -> sneakyTwo(false));
        chooser.addCmd("Stinky One Left", () -> stinkyOne(true));
        chooser.addCmd("Stinky One Right", () -> stinkyOne(false));
        SmartDashboard.putData("autos", chooser);
    }

    /**
     * Returns a command that when scheduled will run the currently selected auto.
     */
    public Command runSelectedAuto() {
        return chooser.selectedCommandScheduler();
    }

    private AutoRoutine olReliable(boolean left) {
        AutoRoutine routine = factory.newRoutine("Ol' Reliable");

        AutoTrajectory startToJ = routine.trajectory("Start-J", !left);
        AutoTrajectory jToHopper = routine.trajectory("J-Hopper", !left);
        AutoTrajectory hopperToK = routine.trajectory("Hopper-K", !left);
        AutoTrajectory kToHopper = routine.trajectory("K-Hopper", !left);
        AutoTrajectory hopperToL = routine.trajectory("Hopper-L", !left);
        AutoTrajectory lToHopper = routine.trajectory("L-Hopper", !left);

        routine
            .active()
            .onTrue(
                sequence(
                    parallel(selection.selectLevel(4), gooseNeck.setHasCoral(false), startToJ.resetOdometry()),
                    startToJ.spawnCmd()
                )
            );

        Trigger toHopper = routine.anyActive(jToHopper, kToHopper, lToHopper);
        Trigger startHopper = jToHopper.atTime(0.5).or(kToHopper.atTime(0.5)).or(lToHopper.atTime(0.5));

        startHopper.onTrue(routines.intake());
        routine.observe(gooseNeck::hasCoral).onTrue(routines.score(toHopper, () -> true));

        startToJ.active().onTrue(gooseNeck.setHasCoral(true));
        startToJ.chain(jToHopper);
        jToHopper.done().onTrue(waitUntil(gooseNeck::hasCoral).andThen(hopperToK.spawnCmd()));
        hopperToK.chain(kToHopper);
        kToHopper.done().onTrue(waitUntil(gooseNeck::hasCoral).andThen(hopperToL.spawnCmd()));
        hopperToL.chain(lToHopper);

        return routine;
    }

    private Command forPiece(boolean left) {
        return parallel(
            sequence(
                deadline(
                    waitUntil(gooseNeck::noCoral),
                    sequence(factory.trajectoryCmd("Start-J", !left), swerve.stop(false))
                ),
                pickup(left ? J : E, left),
                pickupCycle(left ? K : D, left),
                pickupCycle(left ? L : C, left),
                pickupCycle(left ? A : B, left),
                pickupCycle(left ? B : A, left)
            ),
            sequence(
                routines.score(() -> false, () -> true).until(() -> gooseNeck.noCoral() && robot.safeForGoose()),
                routines.intake()
            ).repeatedly()
        ).beforeStarting(
            parallel(
                either(selection.setRight(), selection.setLeft(), () -> left),
                selection.selectLevel(4),
                gooseNeck.setHasCoral(true)
            )
        );
    }

    private Command sneakyTwo(boolean left) {
        return parallel(
            sequence(
                score(left ? G : H, left),
                avoid(left),
                pickup(G, left),
                avoid(left),
                score(left ? H : G, left),
                avoid(left),
                swerve.stop(false)
            ),
            sequence(
                routines.score(() -> false, () -> true).until(() -> gooseNeck.noCoral() && robot.safeForGoose()),
                routines.intake()
            ).repeatedly()
        ).beforeStarting(
            parallel(
                either(selection.setRight(), selection.setLeft(), () -> left),
                selection.selectLevel(4),
                gooseNeck.setHasCoral(true)
            )
        );
    }

    private Command stinkyOne(boolean left) {
        return sequence(
            deadline(waitSeconds(3.0), swerve.stop(false), routines.stow()),
            parallel(
                sequence(score(left ? G : H, left), avoid(left), swerve.stop(false)),
                sequence(
                    routines.score(() -> false, () -> true).until(() -> gooseNeck.noCoral() && robot.safeForGoose()),
                    routines.stow()
                )
            )
        ).beforeStarting(
            parallel(
                either(selection.setRight(), selection.setLeft(), () -> left),
                selection.selectLevel(4),
                gooseNeck.setHasCoral(true)
            )
        );
    }

    private Command pickupCycle(ReefLocation reefLocation, boolean left) {
        return sequence(score(reefLocation, left), pickup(reefLocation, left));
    }

    private Command score(ReefLocation reefLocation, boolean left) {
        return deadline(
            sequence(
                waitUntil(gooseNeck::hasCoral),
                waitUntil(() -> !swerve.wildlifeConservationProgram() && gooseNeck.noCoral())
            ),
            swerve.repulsorDrive(reefLocation, robot::readyToScore, selection::isL4),
            either(selection.setLeft(), selection.setRight(), () -> reefLocation.left)
        );
    }

    private Command pickup(ReefLocation start, boolean left) {
        Timer timer = new Timer();

        return swerve
            .repulsorDrive(
                () -> {
                    var sample = start.back ? FieldConstants.kStationBackwards : FieldConstants.kStationForwards;
                    if (Alliance.isRed()) sample = sample.flipped();
                    if (left) sample = sample.mirrored();

                    if (!timer.hasElapsed(kIntakeRotDelay.value())) {
                        return new Pose2d(
                            sample.x,
                            sample.y,
                            Alliance.isBlue() ? start.side : start.side.rotateBy(Rotation2d.kPi)
                        );
                    } else {
                        return sample.getPose();
                    }
                },
                kIntakeSlowdown::value
            )
            .until(() -> intake.coralDetected() || gooseNeck.hasCoral())
            .beforeStarting(timer::restart);
    }

    private Command avoid(boolean left) {
        return swerve.repulsorDrive(
            () -> {
                var sample = FieldConstants.kAvoidLocation;
                if (Alliance.isRed()) sample = sample.flipped();
                if (left) sample = sample.mirrored();
                return sample.getPose();
            },
            kAvoidSlowdown::value,
            kAvoidTolerance::value
        );
    }
}
