package org.team340.robot;

import choreo.trajectory.SwerveSample;
import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.List;
import org.team340.robot.util.RepulsorField.HorizontalObstacle;
import org.team340.robot.util.RepulsorField.LineObstacle;
import org.team340.robot.util.RepulsorField.Obstacle;
import org.team340.robot.util.RepulsorField.TeardropObstacle;
import org.team340.robot.util.RepulsorField.VerticalObstacle;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 */
public final class Constants {

    public static final double kVoltage = 12.0;

    public static final int kDriver = 0;
    public static final int kCoDriver = 1;

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

    public static final class LowerCAN {

        // *************** Lower CAN Bus ***************

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

    public static final class UpperCAN {

        //*************** Upper CAN Bus ***************

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

    public static final class FieldConstants {

        public static final double kLength = 17.548;
        public static final double kWidth = 8.052;

        public static final double kPipeOffsetY = 0.164;

        public static final Translation2d kBlueLeftCorner = new Translation2d(0.0, kWidth);
        public static final Translation2d kBlueRightCorner = new Translation2d(0.0, 0.0);
        public static final Translation2d kRedLeftCorner = new Translation2d(kLength, 0.0);
        public static final Translation2d kRedRightCorner = new Translation2d(kLength, kWidth);

        public static final Translation2d kReefCenterBlue = new Translation2d(4.489, kWidth / 2.0);
        public static final Translation2d kReefCenterRed = ChoreoAllianceFlipUtil.flip(kReefCenterBlue);

        public static final double kReefCenterToWallDistance = 0.781;

        public static enum ReefLocation {
            A(Rotation2d.fromDegrees(0.0), true, false),
            B(Rotation2d.fromDegrees(0.0), false, false),
            C(Rotation2d.fromDegrees(60.0), true, false),
            D(Rotation2d.fromDegrees(60.0), false, false),
            E(Rotation2d.fromDegrees(120.0), true, true),
            F(Rotation2d.fromDegrees(120.0), false, true),
            G(Rotation2d.fromDegrees(180.0), true, true),
            H(Rotation2d.fromDegrees(180.0), false, true),
            I(Rotation2d.fromDegrees(-120.0), true, true),
            J(Rotation2d.fromDegrees(-120.0), false, true),
            K(Rotation2d.fromDegrees(-60.0), true, false),
            L(Rotation2d.fromDegrees(-60.0), false, false);

            public final Rotation2d side;
            public final boolean left;
            public final boolean back;

            private ReefLocation(Rotation2d side, boolean left, boolean back) {
                this.side = side;
                this.left = left;
                this.back = back;
            }
        }

        private static final double kCoralX = 1.6;
        private static final double kCoralY = 1.125;

        public static final List<Obstacle> kObstacles = List.of(
            // Walls
            new HorizontalObstacle(0.0, 0.15, true),
            new HorizontalObstacle(kWidth, 0.15, false),
            new VerticalObstacle(0.0, 0.15, true),
            new VerticalObstacle(kLength, 0.15, false),
            // Reef
            new TeardropObstacle(kReefCenterBlue, 1.0, 2.5, 0.83, 3.0, 2.0, true),
            new TeardropObstacle(kReefCenterRed, 1.0, 2.5, 0.83, 3.0, 2.0, true),
            // Coral stations
            new LineObstacle(new Translation2d(0.0, kCoralY), new Translation2d(kCoralX, 0.0), 0.15, true),
            new LineObstacle(new Translation2d(0.0, kWidth - kCoralY), new Translation2d(kCoralX, kWidth), 0.15, true),
            new LineObstacle(
                new Translation2d(kLength, kCoralY),
                new Translation2d(kLength - kCoralX, 0.0),
                0.15,
                true
            ),
            new LineObstacle(
                new Translation2d(kLength, kWidth - kCoralY),
                new Translation2d(kLength - kCoralX, kWidth),
                0.15,
                true
            )
        );

        private static double kStationX = 1.58;
        private static double kStationY = 0.63;

        public static final SwerveSample kStationForwards = new SwerveSample(
            0.0,
            kStationX,
            kStationY,
            Math.toRadians(-36.0),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            new double[0],
            new double[0]
        );

        public static final SwerveSample kStationBackwards = new SwerveSample(
            0.0,
            kStationX,
            kStationY,
            Math.toRadians(144.0),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            new double[0],
            new double[0]
        );

        public static final SwerveSample kAvoidLocation = new SwerveSample(
            0.0,
            6.45,
            0.9,
            Math.toRadians(180.0),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            new double[0],
            new double[0]
        );
    }
}
