package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
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
        threeScore();
        l4x3BabyBird();
    }

    public void threeScore() {
        AutoRoutine routine = factory.newRoutine("threeScore");

        AutoTrajectory startToJ = routine.trajectory("threeScore", 0);
        AutoTrajectory jToPickup = routine.trajectory("threeScore", 1);
        AutoTrajectory pickupToK = routine.trajectory("threeScore", 2);
        AutoTrajectory kToPickup = routine.trajectory("threeScore", 3);
        AutoTrajectory pickupToL = routine.trajectory("threeScore", 4);
        AutoTrajectory lToPickup = routine.trajectory("threeScore", 5);

        routine
            .active()
            .onTrue(
                sequence(
                    parallel(
                        selection.selectLevel(4),
                        gooseNeck.setHasCoral(true),
                        startToJ.resetOdometry(),
                        swerve.resetAutoPID()
                    ),
                    startToJ.spawnCmd()
                )
            );

        Trigger toScore = routine.anyActive(startToJ, pickupToK, pickupToL);
        Trigger toIntake = routine.anyActive(jToPickup, kToPickup);
        Trigger startIntake = jToPickup.atTime(0.75).or(kToPickup.atTime(0.75));

        toScore.onTrue(
            routines
                .intake()
                .andThen(new ScheduleCommand(routines.score(toIntake, () -> true, ElevatorPosition.kBabyBird)))
        );
        startIntake.onTrue(routines.intake());

        final double kScoreDelaySeconds = 0.5;
        final double kIntakeDelaySeconds = 0.5;

        startToJ.doneDelayed(kScoreDelaySeconds).onTrue(jToPickup.cmd());
        jToPickup.done().onTrue(waitUntil(gooseNeck::beamBroken).andThen(pickupToK.spawnCmd()));
        pickupToK.doneDelayed(kScoreDelaySeconds).onTrue(kToPickup.cmd());
        kToPickup.done().onTrue(waitUntil(gooseNeck::beamBroken).andThen(pickupToL.spawnCmd()));
        pickupToL.doneDelayed(kScoreDelaySeconds).onTrue(lToPickup.cmd());

        GRRDashboard.addAuto(routine, routine.trajectory("threeScore"));
    }

    public void l4x3BabyBird() {
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
        lToEnd.done().onTrue(routines.stow());

        GRRDashboard.addAuto(routine, List.of(startToJ, jToBird, birdToK, kToBird, birdToL, lToEnd));
    }
}
