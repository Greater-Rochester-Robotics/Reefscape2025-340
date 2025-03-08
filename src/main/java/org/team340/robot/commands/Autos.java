package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import org.team340.lib.util.GRRDashboard;
import org.team340.robot.Robot;
import org.team340.robot.subsystems.Elevator;
import org.team340.robot.subsystems.Elevator.ElevatorPosition;
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

    private final Robot robot;

    private final Elevator elevator;
    private final GooseNeck gooseNeck;
    private final Intake intake;
    private final Lights lights;
    private final Swerve swerve;

    private final Routines routines;
    private final ReefSelection selection;

    private final AutoFactory factory;

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

        // Add autonomous modes to the dashboard
        l4x3BabyBird();
        l4x3Hopper();
    }

    private void l4x3BabyBird() {
        AutoRoutine routine = factory.newRoutine("L4 x3 (Baby Bird)");

        AutoTrajectory startToJ = routine.trajectory("Start-J");
        AutoTrajectory jToBird = routine.trajectory("J-Bird");
        AutoTrajectory birdToK = routine.trajectory("Bird-K");
        AutoTrajectory kToBird = routine.trajectory("K-Bird");
        AutoTrajectory birdToL = routine.trajectory("Bird-L");
        AutoTrajectory lToEnd = routine.trajectory("L-End");

        routine
            .active()
            .onTrue(
                sequence(
                    parallel(
                        selection.selectLevel(4),
                        gooseNeck.setHasCoral(false),
                        startToJ.resetOdometry(),
                        swerve.resetAutoPID()
                    ),
                    startToJ.spawnCmd()
                )
            );

        Trigger toBird = routine.anyActive(jToBird, kToBird);
        Trigger startBird = jToBird.atTime(0.75).or(kToBird.atTime(0.75));

        startBird.onTrue(routines.babyBird(() -> true));
        routine.observe(gooseNeck::hasCoral).onTrue(routines.score(toBird, () -> true, ElevatorPosition.kBabyBird));

        startToJ.active().onTrue(gooseNeck.setHasCoral(true));
        startToJ.chain(jToBird);
        jToBird.done().onTrue(waitUntil(gooseNeck::hasCoral).andThen(birdToK.spawnCmd()));
        birdToK.chain(kToBird);
        kToBird.done().onTrue(waitUntil(gooseNeck::hasCoral).andThen(birdToL.spawnCmd()));
        birdToL.chain(lToEnd);
        lToEnd.done().onTrue(routines.stow(ElevatorPosition.kDown));

        GRRDashboard.addAuto(routine, List.of(startToJ, jToBird, birdToK, kToBird, birdToL, lToEnd));
    }

    private void l4x3Hopper() {
        AutoRoutine routine = factory.newRoutine("L4 x3 (Hopper)");

        AutoTrajectory startToJ = routine.trajectory("Start-J");
        AutoTrajectory jToHopper = routine.trajectory("J-Hopper");
        AutoTrajectory HopperToK = routine.trajectory("Hopper-K");
        AutoTrajectory kToHopper = routine.trajectory("K-Hopper");
        AutoTrajectory HopperToL = routine.trajectory("Hopper-L");
        AutoTrajectory lToEnd = routine.trajectory("L-End");

        routine
            .active()
            .onTrue(
                sequence(
                    parallel(
                        selection.selectLevel(4),
                        gooseNeck.setHasCoral(false),
                        startToJ.resetOdometry(),
                        swerve.resetAutoPID()
                    ),
                    startToJ.spawnCmd()
                )
            );

        Trigger toHopper = routine.anyActive(jToHopper, kToHopper);
        Trigger startHopper = jToHopper.atTime(0.5).or(kToHopper.atTime(0.5));

        startHopper.onTrue(routines.intake(() -> true));
        routine.observe(gooseNeck::hasCoral).onTrue(routines.score(toHopper, () -> true));

        startToJ.active().onTrue(gooseNeck.setHasCoral(true));
        startToJ.chain(jToHopper);
        jToHopper.done().onTrue(waitUntil(gooseNeck::hasCoral).andThen(HopperToK.spawnCmd()));
        HopperToK.chain(kToHopper);
        kToHopper.done().onTrue(waitUntil(gooseNeck::hasCoral).andThen(HopperToL.spawnCmd()));
        HopperToL.chain(lToEnd);
        lToEnd.done().onTrue(routines.stow(ElevatorPosition.kDown));

        GRRDashboard.addAuto(routine, List.of(startToJ, jToHopper, HopperToK, kToHopper, HopperToL, lToEnd));
    }
}
