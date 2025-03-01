package org.team340.lib.swerve;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

@CustomLoggerFor(SwerveAPI.class)
public class SwerveAPILogger extends ClassSpecificLogger<SwerveAPI> {

    public SwerveAPILogger() {
        super(SwerveAPI.class);
    }

    @Override
    public void update(EpilogueBackend backend, SwerveAPI api) {
        logState(backend.getNested("state"), api.state);

        var hardware = backend.getNested("hardware");
        var errorHandler = Epilogue.getConfig().errorHandler;

        api.imu.log(hardware.getNested("imu"), errorHandler);
        for (var m : api.modules) {
            var module = hardware.getNested(m.getName());
            m.moveMotor.log(module.getNested("moveMotor"), errorHandler);
            m.turnMotor.log(module.getNested("turnMotor"), errorHandler);
            m.encoder.log(module.getNested("encoder"), errorHandler);
        }
    }

    private void logState(EpilogueBackend backend, SwerveState state) {
        backend.log("speeds", state.speeds, ChassisSpeeds.struct);
        backend.log("velocity", state.velocity);
        backend.log("pose", state.pose, Pose2d.struct);

        var modules = backend.getNested("modules");
        modules.log("positions", state.modules.positions, SwerveModulePosition.struct);
        modules.log("states", state.modules.states, SwerveModuleState.struct);
        modules.log("lastTarget", state.modules.lastTarget, SwerveModuleState.struct);

        var imu = backend.getNested("imu");
        imu.log("yaw", state.imu.yaw, Rotation2d.struct);
        imu.log("pitch", state.imu.pitch, Rotation2d.struct);
        imu.log("roll", state.imu.roll, Rotation2d.struct);

        var odometryThread = backend.getNested("odometryThread");
        odometryThread.log("timesync", state.odometryThread.timesync);
        odometryThread.log("successes", state.odometryThread.successes);
        odometryThread.log("failures", state.odometryThread.failures);
    }
}
