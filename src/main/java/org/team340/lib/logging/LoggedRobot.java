package org.team340.lib.logging;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.hal.NotifierJNI;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotController;
import java.util.function.Consumer;
import org.team340.lib.util.DisableWatchdog;
import org.team340.lib.util.Tunable;

/**
 * LoggedRobot implements the IterativeRobotBase robot program framework,
 * and is intended to be subclassed by a user creating a robot program.
 *
 * <p>The LoggedRobot class configures the program for logging data to NetworkTables
 * via Epilogue, which is captured by the {@link DataLogManager} alongside Driver
 * Station values and saved to file. Loop timings are also recorded using the
 * {@link Profiler} class.
 *
 * <p>{@code periodic()} functions from the base class are called on an interval
 * by a Notifier instance. Additionally, utility classes with periodic methods
 * such as {@link Tunable} are updated on the same interval.
 */
public class LoggedRobot extends IterativeRobotBase {

    public static final double DEFAULT_PERIOD = 0.02;

    private final int notifier = NotifierJNI.initializeNotifier();
    private final Consumer<EpilogueBackend> logger;

    private long nextCycle = 0L;

    /**
     * Constructor for LoggedRobot.
     */
    protected LoggedRobot() {
        this(DEFAULT_PERIOD);
    }

    /**
     * Constructor for LoggedRobot.
     * @param period Period in seconds.
     */
    protected LoggedRobot(double period) {
        super(period);
        NotifierJNI.setNotifierName(notifier, "LoggedRobot");

        DriverStation.silenceJoystickConnectionWarning(true);
        DisableWatchdog.in(this, "m_watchdog");

        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        SignalLogger.enableAutoLogging(false);

        EpilogueProxy.getConfig().root = "/Telemetry";
        logger = EpilogueProxy.getLogger(this);
    }

    @Override
    public void startCompetition() {
        robotInit();
        if (isSimulation()) {
            simulationInit();
        }

        // Tell the DS that the robot is ready to be enabled
        System.out.println("********** Robot program startup complete **********");
        DriverStationJNI.observeUserProgramStarting();

        // Loop forever, calling the appropriate mode-dependent function
        while (true) {
            long now = RobotController.getFPGATime();
            if (nextCycle < now) {
                nextCycle = now;
            } else {
                NotifierJNI.updateNotifierAlarm(notifier, nextCycle);
                if (NotifierJNI.waitForNotifierAlarm(notifier) == 0L) {
                    break;
                }
            }

            nextCycle += (long) (getPeriod() * 1000000.0);

            Profiler.start("robot");
            Profiler.run("loopFunction", () -> loopFunc());
            Profiler.run("epilogue", () -> logger.accept(EpilogueProxy.getRootBackend()));
            Profiler.run("tunables", Tunable::update);
            Profiler.end();
        }
    }

    @Override
    public void endCompetition() {
        NotifierJNI.stopNotifier(notifier);
    }

    @Override
    public void close() {
        NotifierJNI.stopNotifier(notifier);
        NotifierJNI.cleanNotifier(notifier);
    }

    @Override
    public void robotPeriodic() {}

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
