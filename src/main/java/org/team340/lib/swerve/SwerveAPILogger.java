package org.team340.lib.swerve;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import org.team340.lib.logging.EpilogueProxy;

@CustomLoggerFor(SwerveAPI.class)
public class SwerveAPILogger extends ClassSpecificLogger<SwerveAPI> {

    private static final Map<SwerveAPI, Consumer<EpilogueBackend>> cache = new HashMap<>();

    public SwerveAPILogger() {
        super(SwerveAPI.class);
    }

    @Override
    public void update(EpilogueBackend backend, SwerveAPI api) {
        logState(backend.getNested("state"), api.state);

        cache
            .computeIfAbsent(api, key -> {
                var imu = EpilogueProxy.getLogger(api.imu.getAPI());

                Map<String, Consumer<EpilogueBackend>> modules = new HashMap<>();
                for (SwerveModule module : api.modules) {
                    var move = EpilogueProxy.getLogger(module.moveMotor.getAPI());
                    var turn = EpilogueProxy.getLogger(module.turnMotor.getAPI());
                    var encoder = EpilogueProxy.getLogger(module.encoder.getAPI());

                    modules.put(module.getName(), b -> {
                        move.accept(b.getNested("moveMotor"));
                        turn.accept(b.getNested("turnMotor"));
                        encoder.accept(b.getNested("encoder"));
                    });
                }

                return b -> {
                    imu.accept(b.getNested("imu"));
                    for (var module : modules.entrySet()) {
                        module.getValue().accept(b.getNested(module.getKey()));
                    }
                };
            })
            .accept(backend.getNested("hardware"));
    }

    private void logState(EpilogueBackend backend, SwerveState state) {
        backend.log("speeds", state.speeds, ChassisSpeeds.struct);
        backend.log("velocity", state.velocity);
        backend.log("pose", state.pose, Pose2d.struct);
        backend.log("pitch", state.pitch, Rotation2d.struct);
        backend.log("roll", state.roll, Rotation2d.struct);
        backend.log("odometryPose", state.odometryPose, Pose2d.struct);

        var modules = backend.getNested("modules");
        modules.log("positions", state.modules.positions, SwerveModulePosition.struct);
        modules.log("states", state.modules.states, SwerveModuleState.struct);
        modules.log("lastTarget", state.modules.lastTarget, SwerveModuleState.struct);

        var odometryThread = backend.getNested("odometryThread");
        odometryThread.log("timesync", state.odometryThread.timesync);
        odometryThread.log("successes", state.odometryThread.successes);
        odometryThread.log("failures", state.odometryThread.failures);
    }
}
