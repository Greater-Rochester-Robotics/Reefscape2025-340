package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
        chooser.addRoutine("Left L4 x3 (Hopper)", () -> l4x3Hopper(false));
        chooser.addRoutine("Right L4 x3 (Hopper)", () -> l4x3Hopper(true));
        chooser.addRoutine("Left L4 x3 (Baby Bird)", () -> l4x3BabyBird(false));
        chooser.addRoutine("Right L4 x3 (Baby Bird)", () -> l4x3BabyBird(true));
        SmartDashboard.putData("autos", chooser);
    }

    /**
     * Returns a command that when scheduled will run the currently selected auto.
     */
    public Command runSelectedAuto() {
        return chooser.selectedCommandScheduler();
    }

    private AutoRoutine l4x3Hopper(boolean mirror) {
        AutoRoutine routine = factory.newRoutine("L4 x3 (Hopper)");

        AutoTrajectory startToJ = routine.trajectory("Start-J", mirror);
        AutoTrajectory jToHopper = routine.trajectory("J-Hopper", mirror);
        AutoTrajectory hopperToK = routine.trajectory("Hopper-K", mirror);
        AutoTrajectory kToHopper = routine.trajectory("K-Hopper", mirror);
        AutoTrajectory hopperToL = routine.trajectory("Hopper-L", mirror);
        AutoTrajectory lToHopper = routine.trajectory("L-Hopper", mirror);

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

    private AutoRoutine l4x3BabyBird(boolean mirror) {
        AutoRoutine routine = factory.newRoutine("L4 x3 (Baby Bird)");

        AutoTrajectory startToJ = routine.trajectory("Start-J", mirror);
        AutoTrajectory jToBird = routine.trajectory("J-Bird", mirror);
        AutoTrajectory birdToK = routine.trajectory("Bird-K", mirror);
        AutoTrajectory kToBird = routine.trajectory("K-Bird", mirror);
        AutoTrajectory birdToL = routine.trajectory("Bird-L", mirror);
        AutoTrajectory lToHopper = routine.trajectory("L-Hopper", mirror);

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

        Trigger toIntake = routine.anyActive(jToBird, kToBird, lToHopper);
        Trigger startBird = jToBird.atTime(0.75).or(kToBird.atTime(0.75));

        startBird.onTrue(routines.babyBird(() -> true));
        routine.observe(gooseNeck::hasCoral).onTrue(routines.score(toIntake, () -> true, ElevatorPosition.kBabyBird));

        startToJ.active().onTrue(gooseNeck.setHasCoral(true));
        startToJ.chain(jToBird);
        jToBird.done().onTrue(waitUntil(gooseNeck::hasCoral).andThen(birdToK.spawnCmd()));
        birdToK.chain(kToBird);
        kToBird.done().onTrue(waitUntil(gooseNeck::hasCoral).andThen(birdToL.spawnCmd()));
        birdToL.chain(lToHopper);
        lToHopper.atTime(0.75).onTrue(routines.intake());

        return routine;
    }
}
