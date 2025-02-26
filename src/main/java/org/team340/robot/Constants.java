package org.team340.robot;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 */
public final class Constants {

    public static final double kVoltage = 12.0;

    public static final int kDriver = 0;
    public static final int kCoDriver = 1;

    public static final class FieldConstants {

        public static final double kLength = 17.548;
        public static final double kWidth = 8.052;

        public static final Translation2d kBlueLeftCorner = new Translation2d(0.0, kWidth);
        public static final Translation2d kBlueRightCorner = new Translation2d(0.0, 0.0);
        public static final Translation2d kRedLeftCorner = new Translation2d(kLength, 0.0);
        public static final Translation2d kRedRightCorner = new Translation2d(kLength, kWidth);

        public static final Translation2d kReefCenterBlue = new Translation2d(4.489, kWidth / 2.0);
        public static final Translation2d kReefCenterRed = ChoreoAllianceFlipUtil.flip(kReefCenterBlue);

        public static final double kPipeOffsetX = -0.681;
        public static final double kPipeOffsetY = -0.164;

        public static final double kReefCenterToWallDistance = 0.781;
    }

    public final class Cameras {

        public static final Transform3d kMiddle = new Transform3d(
            new Translation3d(0.354, 0.0, 0.215),
            new Rotation3d(0.0, Math.toRadians(-5.0), Math.toRadians(0.0))
        );
        public static final Transform3d kLeft = new Transform3d(
            new Translation3d(0.316, 0.092, 0.211),
            new Rotation3d(0.0, Math.toRadians(-5.0), Math.toRadians(45.0))
        );
        public static final Transform3d kRight = new Transform3d(
            new Translation3d(0.316, -0.092, 0.211),
            new Rotation3d(0.0, Math.toRadians(-5.0), Math.toRadians(-45.0))
        );
    }

    /**
     * The RobotMap class defines CAN IDs, CAN bus names, DIO/PWM/PH/PCM channel
     * IDs, and other relevant identifiers for addressing robot hardware.
     */
    public static final class LowerCAN {

        // upper CAN bus is the default CAN bus.
        // Talon FX default constructor uses "" and we can't change it.
        public static final String kLowerCANBus = "LowerCAN";

        // *************** Lower CAN Bus ***************

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

        public static final int kCanandgyro = 14;

        // Elevator
        public static final int kElevatorLead = 20;
        public static final int kElevatorFollow = 21;
        public static final int kElevatorCANdi = 22;
    }

    public static final class UpperCAN {

        //*************** Upper CAN Bus ***************

        // Goose
        public static final int kGooseNeckMotor = 30;
        public static final int kGooseBeakMotor = 31;
        public static final int kGooseCANdi = 32;

        // Intake
        public static final int kIntakeMotor = 40;

        // Climber
        public static final int kClimberMotor = 50;
    }

    public static final class RioIO {

        public static final int kIntakeBeamBreak = 9;
        public static final int kLights = 9;

        public static final int kClimberLimitSwitch = 0;
    }
}
