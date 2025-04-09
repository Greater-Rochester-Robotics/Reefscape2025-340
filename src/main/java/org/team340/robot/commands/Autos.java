package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static org.team340.robot.Constants.FieldConstants.ReefLocation.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
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

    private static final TunableDouble kIntakeSlowdown = Tunable.doubleValue("autos/kIntakeSlowdown", 0.5);

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
        parallel(
            sequence(
                pickupCycle(left ? I : F, left),
                pickupCycle(left ? J : E, left),
                pickupCycle(left ? K : D, left),
                pickupCycle(left ? L : C, left)
            ),
            sequence(routines.score(() -> false, () -> true), routines.intake()).repeatedly()
        ).beforeStarting(parallel(selection.selectLevel(4), gooseNeck.setHasCoral(true)));
        return none();
    }

    private Command pickupCycle(ReefLocation reefLocation, boolean left) {
        return sequence(
            parallel(
                swerve.repulsorDrive(reefLocation, robot::readyToScore).until(gooseNeck::noCoral),
                either(selection.setLeft(), selection.setRight(), () -> reefLocation.left)
            ),
            swerve
                .repulsorDrive(
                    () -> {
                        var sample = FieldConstants.kStationSample;
                        if (Alliance.isBlue()) sample = sample.flipped();
                        if (left) sample = sample.mirrored();
                        return sample.getPose();
                    },
                    kIntakeSlowdown::value
                )
                .until(intake::coralDetected)
        );
    }
}
