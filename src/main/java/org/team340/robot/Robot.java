package org.team340.robot;

import static edu.wpi.first.wpilibj.XboxController.Axis.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team340.lib.logging.LoggedRobot;
import org.team340.lib.logging.Profiler;
import org.team340.lib.util.DisableWatchdog;
import org.team340.robot.commands.Autos;
import org.team340.robot.commands.Routines;
import org.team340.robot.subsystems.Climber;
import org.team340.robot.subsystems.Elevator;
import org.team340.robot.subsystems.Elevator.ElevatorPosition;
import org.team340.robot.subsystems.GooseNeck;
import org.team340.robot.subsystems.Intake;
import org.team340.robot.subsystems.Swerve;
import org.team340.robot.util.ReefSelection;

@Logged
public final class Robot extends LoggedRobot {

    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    public final Climber climber;
    public final Elevator elevator;
    public final GooseNeck gooseNeck;
    public final Intake intake;
    // public final Lights lights;
    public final Swerve swerve;

    public final ReefSelection selection;

    public final Routines routines;
    public final Autos autos;

    private final CommandXboxController driver;
    private final CommandXboxController coDriver;

    public Robot() {
        // Initialize subsystems
        climber = new Climber();
        elevator = new Elevator();
        gooseNeck = new GooseNeck();
        intake = new Intake();
        // lights = new Lights();
        swerve = new Swerve();

        // Initialize helpers
        selection = new ReefSelection();

        // Initialize compositions
        routines = new Routines(this);
        autos = new Autos(this);

        // Initialize controllers
        driver = new CommandXboxController(Constants.DRIVER);
        coDriver = new CommandXboxController(Constants.CO_DRIVER);

        // Setup lights
        // lights.disabled().until(this::isEnabled).schedule();
        // RobotModeTriggers.disabled().whileTrue(lights.disabled());
        // RobotModeTriggers.autonomous().whileTrue(lights.sides.flames());
        // RobotModeTriggers.teleop().whileTrue(lights.sides.levelSelection(selection));
        // new Trigger(this::isEnabled)
        //     .and(gooseNeck::hasCoral)
        //     .onTrue(lights.top.hasCoral(gooseNeck::goosing, gooseNeck::getPosition, selection))
        //     .onFalse(lights.top.scored().onlyIf(this::isEnabled));

        // Create triggers
        Trigger allowGoosing = coDriver.a().negate();
        Trigger changedReference = new Trigger(swerve::changedReference);

        // Set default commands
        elevator.setDefaultCommand(elevator.goTo(ElevatorPosition.DOWN, this::safeForGoose));
        gooseNeck.setDefaultCommand(gooseNeck.stow(this::safeForGoose));
        swerve.setDefaultCommand(swerve.drive(this::driverX, this::driverY, this::driverAngular));

        // Driver bindings
        driver.a().onTrue(routines.intake(driver.a()));
        driver.b().whileTrue(routines.swallow());
        driver.x().whileTrue(routines.barf());
        driver.y().onTrue(none()); // Reserved (Force goose spit)

        driver.leftStick().whileTrue(swerve.turboSpin(this::driverX, this::driverY, this::driverAngular));
        driver.axisLessThan(kRightY.value, -0.5).onTrue(selection.incrementLevel());
        driver.axisGreaterThan(kRightY.value, 0.5).onTrue(selection.decrementLevel());

        driver.povUp().whileTrue(routines.barf());
        driver.povDown().whileTrue(routines.swallow());
        driver.povLeft().onTrue(swerve.tareRotation());

        driver.leftBumper().onTrue(selection.setLeft()).whileTrue(routines.assistedScore(driver.y(), allowGoosing));
        driver.rightBumper().onTrue(selection.setRight()).whileTrue(routines.assistedScore(driver.y(), allowGoosing));

        changedReference.and(RobotModeTriggers.teleop()).onTrue(setDriverRumble(1.0).withTimeout(0.15));

        // Co-driver bindings
        coDriver.a().onTrue(none()); // Reserved (No goosing around)
        coDriver.b().whileTrue(climber.climb());
        coDriver.x().whileTrue(climber.deploy());
        coDriver.y().whileTrue(climber.override());

        coDriver.leftStick().and(coDriver.rightStick()).toggleOnTrue(climber.coastMode());

        coDriver.povUp().onTrue(selection.incrementLevel());
        coDriver.povDown().onTrue(selection.decrementLevel());

        coDriver.leftBumper().and(coDriver.rightBumper()).toggleOnTrue(routines.killTheGoose());

        // Disable loop overrun warnings from the command
        // scheduler, since we already log loop timings
        DisableWatchdog.in(scheduler, "m_watchdog");
    }

    /**
     * Returns {@code true} if it is safe for the goose neck and elevator to move.
     */
    public boolean safeForGoose() {
        return !gooseNeck.beamBroken() && swerve.wildlifeConservationProgram();
    }

    /**
     * Returns {@code true} if the robot is ready to score.
     */
    public boolean readyToScore() {
        return (
            swerve.wildlifeConservationProgram() &&
            (Robot.isSimulation() || (gooseNeck.hasCoral() && elevator.atPosition() && elevator.scoring()))
        );
    }

    /**
     * Returns the current match time in seconds.
     */
    public double matchTime() {
        return Math.max(0.0, DriverStation.getMatchTime());
    }

    @NotLogged
    public double driverX() {
        return driver.getLeftX();
    }

    @NotLogged
    public double driverY() {
        return driver.getLeftY();
    }

    @NotLogged
    public double driverAngular() {
        return driver.getLeftTriggerAxis() - driver.getRightTriggerAxis();
    }

    /**
     * Returns a command that sets the rumble output of the driver's controller.
     * @param value The normalized value (0 to 1) to set the rumble to.
     */
    private Command setDriverRumble(double value) {
        return run(() -> driver.setRumble(RumbleType.kBothRumble, value))
            .finallyDo(() -> driver.setRumble(RumbleType.kBothRumble, 0.0))
            .ignoringDisable(true)
            .withName("Robot.setDriverRumble(" + value + ")");
    }

    @Override
    public void robotPeriodic() {
        Profiler.run("scheduler", scheduler::run);
        // Profiler.run("lights", lights::update);
    }
}
