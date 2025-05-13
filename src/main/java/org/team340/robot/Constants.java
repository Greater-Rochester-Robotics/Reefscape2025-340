package org.team340.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.team340.robot.util.Vision.CameraConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 */
public final class Constants {

    public static final double kVoltage = 12.0;

    public static final int kDriver = 0;
    public static final int kCoDriver = 1;

    public static final CameraConfig[] kCameras = {
        new CameraConfig(
            "middle",
            new Translation3d(0.354, 0.0, 0.215),
            new Rotation3d(0.0, Math.toRadians(-5.0), Math.toRadians(0.0))
        ),
        new CameraConfig(
            "left",
            new Translation3d(0.316, 0.092, 0.211),
            new Rotation3d(0.0, Math.toRadians(-5.0), Math.toRadians(45.0))
        ),
        new CameraConfig(
            "right",
            new Translation3d(0.316, -0.092, 0.211),
            new Rotation3d(0.0, Math.toRadians(-5.0), Math.toRadians(-45.0))
        )
    };

    public static final class LowerCAN {

        public static final String kLowerCANBus = "LowerCAN";

        // Swerve
        public static final int kFlMove = 2;
        public static final int kFlTurn = 3;
        public static final int kFrMove = 4;
        public static final int kFrTurn = 5;
        public static final int kBlMove = 6;
        public static final int kBlTurn = 7;
        public static final int kBrMove = 8;
        public static final int kBrTurn = 9;

        public static final int kFlEncoder = 10;
        public static final int kFrEncoder = 11;
        public static final int kBlEncoder = 12;
        public static final int kBrEncoder = 13;

        // Elevator
        public static final int kElevatorLead = 20;
        public static final int kElevatorFollow = 21;
        public static final int kElevatorCANdi = 22;
    }

    public static final class RioCAN {

        // Swerve
        public static final int kCanandgyro = 14;

        // Goose
        public static final int kGooseNeckMotor = 30;
        public static final int kGooseBeakMotor = 31;
        public static final int kGooseCANdi = 32;

        // Intake
        public static final int kIntakeMotor = 40;
        public static final int kIntakeCANrange = 41;

        // Climber
        public static final int kClimberMotor = 50;
    }

    public static final class RioIO {

        public static final int kIntakeBeamBreak = 9;
        public static final int kLights = 9;
    }
}
