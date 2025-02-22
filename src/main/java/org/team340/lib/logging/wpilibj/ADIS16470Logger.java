package org.team340.lib.logging.wpilibj;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;

@CustomLoggerFor(ADIS16470_IMU.class)
public class ADIS16470Logger extends ClassSpecificLogger<ADIS16470_IMU> {

    public ADIS16470Logger() {
        super(ADIS16470_IMU.class);
    }

    @Override
    public void update(EpilogueBackend backend, ADIS16470_IMU adis16470) {
        backend.log("yaw", adis16470.getAngle(IMUAxis.kYaw));
        backend.log("pitch", adis16470.getAngle(IMUAxis.kPitch));
        backend.log("roll", adis16470.getAngle(IMUAxis.kRoll));
    }
}
