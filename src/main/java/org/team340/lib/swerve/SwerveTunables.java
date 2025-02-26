package org.team340.lib.swerve;

import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.util.Tunable;

/**
 * Utility class for constructing tunables for the {@link SwerveAPI}.
 */
final class SwerveTunables {

    private SwerveTunables() {
        throw new AssertionError("This is a utility class!");
    }

    /**
     * Enables publishing tunables for adjustment of the API's constants.
     * @param name The parent name for the tunables in NetworkTables.
     * @param api The swerve API.
     */
    public static void initialize(String name, SwerveAPI api) {
        SwerveConfig config = api.config;

        // setTimings()
        Tunable.doubleValue(name + "/timings/discretization", config.discretizationPeriod, v ->
            config.discretizationPeriod = v
        );

        // setMovePID()
        Tunable.doubleValue(name + "/movePID/kP", config.movePID[0], v -> {
            config.movePID[0] = v;
            reapplyGains(true, api);
        });
        Tunable.doubleValue(name + "/movePID/kI", config.movePID[1], v -> {
            config.movePID[1] = v;
            reapplyGains(true, api);
        });
        Tunable.doubleValue(name + "/movePID/kD", config.movePID[2], v -> {
            config.movePID[2] = v;
            reapplyGains(true, api);
        });

        // setMoveFF()
        Tunable.doubleValue(name + "/moveFF/kS", config.moveFF[0], v -> {
            config.moveFF[0] = v;
            reapplyGains(true, api);
        });
        Tunable.doubleValue(name + "/moveFF/kV", config.moveFF[1], v -> {
            config.moveFF[1] = v;
            reapplyGains(true, api);
        });

        // setTurnPID()
        Tunable.doubleValue(name + "/turnPID/kP", config.turnPID[0], v -> {
            config.turnPID[0] = v;
            reapplyGains(false, api);
        });
        Tunable.doubleValue(name + "/turnPID/kI", config.turnPID[1], v -> {
            config.turnPID[1] = v;
            reapplyGains(false, api);
        });
        Tunable.doubleValue(name + "/turnPID/kD", config.turnPID[2], v -> {
            config.turnPID[2] = v;
            reapplyGains(false, api);
        });

        // setLimits()
        Tunable.doubleValue(name + "/limits/velocity", config.velocity, v -> config.velocity = v);
        Tunable.doubleValue(name + "/limits/velDeadband", config.velDeadband, v -> config.velDeadband = v);
        Tunable.doubleValue(name + "/limits/slipAccel", config.slipAccel, v -> config.slipAccel = v);
        Tunable.doubleValue(name + "/limits/torqueAccel", config.torqueAccel, v -> config.torqueAccel = v);
        Tunable.doubleValue(name + "/limits/angularAccel", config.angularAccel, v -> config.angularAccel = v);

        // setDriverProfile()
        Tunable.doubleValue(name + "/driverProfile/vel", config.driverVel, v -> config.driverVel = v);
        Tunable.doubleValue(name + "/driverProfile/velExp", config.driverVelExp, v -> config.driverVelExp = v);
        Tunable.doubleValue(name + "/driverProfile/velDeadband", config.driverVelDeadband, v ->
            config.driverVelDeadband = v
        );
        Tunable.doubleValue(name + "/driverProfile/angularVel", config.driverAngularVel, v ->
            config.driverAngularVel = v
        );
        Tunable.doubleValue(name + "/driverProfile/angularVelExp", config.driverAngularVelExp, v ->
            config.driverAngularVelExp = v
        );
        Tunable.doubleValue(name + "/driverProfile/angularVelDeadband", config.driverAngularVelDeadband, v ->
            config.driverAngularVelDeadband = v
        );
    }

    /**
     * Re-applies PID and FF gains to motors from the swerve config.
     * Used for setting new gains after the config has been mutated.
     * @param moveMotors {@code true} reapplies to all move motors, {@code false} reapplies to all turn motors.
     * @param api The swerve API.
     */
    private static void reapplyGains(boolean moveMotors, SwerveAPI api) {
        for (var module : api.modules) {
            if (moveMotors) {
                module.moveMotor.reapplyGains();
            } else {
                module.turnMotor.reapplyGains();
            }
        }
    }
}
