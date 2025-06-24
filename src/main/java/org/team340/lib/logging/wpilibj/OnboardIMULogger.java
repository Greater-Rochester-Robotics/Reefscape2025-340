package org.team340.lib.logging.wpilibj;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj.OnboardIMU;

@CustomLoggerFor(OnboardIMU.class)
public class OnboardIMULogger extends ClassSpecificLogger<OnboardIMU> {

    public OnboardIMULogger() {
        super(OnboardIMU.class);
    }

    @Override
    public void update(EpilogueBackend backend, OnboardIMU onboardIMU) {
        backend.log("yaw", onboardIMU.getYawRadians());
        backend.log("pitch", onboardIMU.getAngleX());
        backend.log("roll", onboardIMU.getAngleY());
    }
}
