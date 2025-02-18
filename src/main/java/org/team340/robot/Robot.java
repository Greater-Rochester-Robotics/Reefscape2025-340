package org.team340.robot;

import static edu.wpi.first.wpilibj.XboxController.Axis.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.team340.lib.util.GRRDashboard;
import org.team340.lib.util.Profiler;
import org.team340.lib.util.Tunable;
import org.team340.robot.commands.Autos;
import org.team340.robot.commands.Routines;
import org.team340.robot.subsystems.Elevator;
import org.team340.robot.subsystems.Elevator.ElevatorPosition;
import org.team340.robot.subsystems.GooseNeck;
import org.team340.robot.subsystems.Intake;
import org.team340.robot.subsystems.Lights;
import org.team340.robot.subsystems.Swerve;

@Logged
public final class Robot extends TimedRobot {

    public final Elevator elevator;
    public final GooseNeck gooseNeck;
    public final Intake intake;
    public final Lights lights;
    public final Swerve swerve;

    public final Routines routines;
    public final Autos autos;

    private final CommandXboxController driver;
    private final CommandXboxController coDriver;

    private int selectedLevel = 4;

    public Robot() {
        DriverStation.silenceJoystickConnectionWarning(true);

        // Configure logging
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        SignalLogger.enableAutoLogging(false);
        Epilogue.getConfig().root = "/Telemetry";

        // Initialize subsystems
        elevator = new Elevator();
        gooseNeck = new GooseNeck();
        intake = new Intake();
        lights = new Lights();
        swerve = new Swerve();

        // Initialize compositions
        routines = new Routines(this);
        autos = new Autos(this);

        // Initialize controllers
        driver = new CommandXboxController(Constants.kDriver);
        coDriver = new CommandXboxController(Constants.kCoDriver);

        // Create triggers
        RobotModeTriggers.autonomous().whileTrue(GRRDashboard.runSelectedAuto());

        // Setup lights
        lights.disabled().until(this::isEnabled).schedule();
        RobotModeTriggers.disabled().whileTrue(lights.disabled());
        RobotModeTriggers.autonomous().whileTrue(parallel(lights.sides.flames(), lights.top.off()));
        RobotModeTriggers.teleop().whileTrue(lights.sides.levelSelection(() -> selectedLevel));
        RobotModeTriggers.teleop()
            .and(gooseNeck::hasCoral)
            .onTrue(lights.top.hasCoral(gooseNeck::goosing, gooseNeck::getPosition))
            .onFalse(lights.top.scored().onlyIf(this::isTeleop));

        // Set default commands
        elevator.setDefaultCommand(elevator.goTo(ElevatorPosition.kDown, this::safeForGoose));
        gooseNeck.setDefaultCommand(gooseNeck.stow(this::safeForGoose));
        swerve.setDefaultCommand(
            swerve.drive(
                driver::getLeftX,
                driver::getLeftY,
                () -> driver.getLeftTriggerAxis() - driver.getRightTriggerAxis()
            )
        );

        // Driver bindings
        driver.a().onTrue(routines.intake(driver.a()));
        driver.b().whileTrue(routines.swallow());

        driver
            .x()
            .whileTrue(routines.scoreForward(() -> ElevatorPosition.level(selectedLevel), driver.y(), true, false));
        driver
            .leftBumper()
            .whileTrue(routines.scoreForward(() -> ElevatorPosition.level(selectedLevel), driver.y(), true, true));
        driver
            .rightBumper()
            .whileTrue(routines.scoreForward(() -> ElevatorPosition.level(selectedLevel), driver.y(), false, true));

        driver.axisLessThan(kRightY.value, -0.5).onTrue(incrementLevel());
        driver.axisGreaterThan(kRightY.value, 0.5).onTrue(decrementLevel());

        driver.povLeft().onTrue(swerve.tareRotation());
        driver.povUp().whileTrue(routines.barf());
        driver.povDown().whileTrue(routines.swallow());

        // Co-driver bindings
        coDriver.a().onTrue(none());
    }

    public boolean safeForGoose() {
        return !gooseNeck.beamBroken() && swerve.safeForGoose();
    }

    private Command incrementLevel() {
        return runOnce(() -> selectedLevel = (int) MathUtil.inputModulus(selectedLevel + 1, 0.5, 4.5))
            .ignoringDisable(true)
            .withName("Robot.incrementLevel()");
    }

    private Command decrementLevel() {
        return runOnce(() -> selectedLevel = (int) MathUtil.inputModulus(selectedLevel - 1, 0.5, 4.5))
            .ignoringDisable(true)
            .withName("Robot.decrementLevel()");
    }

    @Override
    public void robotPeriodic() {
        Profiler.start("RobotPeriodic");
        Profiler.run("CommandScheduler", () -> CommandScheduler.getInstance().run());
        Profiler.run("Lights", lights::update);
        Profiler.run("Epilogue", () -> Epilogue.update(this));
        Profiler.run("GRRDashboard", GRRDashboard::update);
        Profiler.run("Tunables", Tunable::update);
        Profiler.end();
    }

    @Override
    public void simulationPeriodic() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testPeriodic() {}
}
