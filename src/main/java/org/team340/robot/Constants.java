package org.team340.robot;

import com.ctre.phoenix6.signals.ForwardLimitSourceValue;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 */
public final class Constants {

    public static final double kVoltage = 12.0;

    public static final int kDriver = 0;
    public static final int kCoDriver = 1;

    /**
     * The RobotMap class defines CAN IDs, CAN bus names, DIO/PWM/PH/PCM channel
     * IDs, and other relevant identifiers for addressing robot hardware.
     */
    public static final class RobotMap {

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
        public static final int kElevatorLead = 15;
        public static final int kElevatorFollow = 16;
        public static final int kElevatorLeadEncoder = 17;
        public static final int kElevatorFollowEncoder = 18;

        // *************** Upper CAN Bus ***************

        // Goose Neck
        public static final int kGooseNeckMotor = 30;
        public static final int kGooseNeckEncoder = 33;

        // Goose Beak
        public static final int kGooseBeakMotor = 31;
        public static final int kGooseBeakCANdi = 32;
        public static final ForwardLimitSourceValue kGooseBeakCANdiPort = ForwardLimitSourceValue.RemoteCANdiS1;

        // Intake
        public static final int kIntakeMotor = 40;
        // This is a DIO channel (not CAN).
        public static final int kIntakeBeamBreak = 9;
    }
}
