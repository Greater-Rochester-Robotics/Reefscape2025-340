package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team340.lib.util.GRRDashboard;
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
        GRRDashboard.setTrajectoryCache(factory.cache());
        GRRDashboard.addAuto("Three Score", threeScore());
    }

    public Command threeScore() {
        AutoRoutine auto = factory.newRoutine("threeScore");

        AutoTrajectory startToJ = auto.trajectory("threeScore", 0);
        AutoTrajectory jToPickup = auto.trajectory("threeScore", 1);
        AutoTrajectory pickupToK = auto.trajectory("threeScore", 2);
        AutoTrajectory kToPickup = auto.trajectory("threeScore", 3);
        AutoTrajectory pickupToL = auto.trajectory("threeScore", 4);
        AutoTrajectory lToPickup = auto.trajectory("threeScore", 5);

        auto
            .active()
            .onTrue(
                sequence(
                    parallel(selection.selectLevel(4), gooseNeck.setHasCoral(true), startToJ.resetOdometry()),
                    startToJ.spawnCmd()
                )
            );

        Trigger toScore = auto.anyActive(startToJ, pickupToK, pickupToL);
        Trigger toIntake = auto.anyActive(jToPickup, kToPickup);
        Trigger startIntake = jToPickup.atTime(0.75).or(kToPickup.atTime(0.75));

        toScore.onTrue(routines.intake().andThen(new ScheduleCommand(routines.score(toIntake))));
        startIntake.onTrue(routines.intake());

        final double kScoreDelaySeconds = 0.5;
        final double kIntakeDelaySeconds = 0.5;

        startToJ.doneDelayed(kScoreDelaySeconds).onTrue(jToPickup.cmd());
        jToPickup.doneDelayed(kIntakeDelaySeconds).onTrue(pickupToK.cmd());
        pickupToK.doneDelayed(kScoreDelaySeconds).onTrue(kToPickup.cmd());
        kToPickup.doneDelayed(kIntakeDelaySeconds).onTrue(pickupToL.cmd());
        pickupToL.doneDelayed(kScoreDelaySeconds).onTrue(lToPickup.cmd());

        return auto.cmd();
    }
}
