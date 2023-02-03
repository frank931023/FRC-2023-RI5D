// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int motorFrontLeft = 2;
        public static final int motorFrontRight = 4;
        public static final int motorRearLeft = 5;
        public static final int motorRearRight = 3;

        public static final double chassisSpeedScaler = 0.5;

        public static final int[] kLeftEncoderPort = {0, 0};
        public static final int[] kRightEncoderPort = {0, 0};
        public static final boolean kLeftEncoderReversed = true;
        public static final boolean kRightEncoderReversed = true;
        public static final double kEncoderCPR = 0;
        public static final double kWheelDiameterMeters = 0;
        public static final double kWheelCircumference = kWheelDiameterMeters * Math.PI;
        public static final double kGearRatio = 0;
        public static final double kDistancePerPulse = kWheelCircumference/kEncoderCPR/kGearRatio;
        // distancePerPulse = gearRatio * wheelPerimeter / EncoderCPR

        // Trajectory Constraints
        public static final int ksVolts = 0;
        public static final int kvVoltSecondsPerMeter = 0;
        public static final int kaVoltSecondsSquaredPerMeter = 0;
        public static final int kPDriveVel = 0;

        // Chassis Related
        public static final double kTrackWidthmeters = 0;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthmeters);

        // Charge Station
        public static final double kLockPIDLeftkP = 0.1;
        public static final double kLockPIDLeftkI = 0.1;
        public static final double kLockPIDLeftkD = 0.01;
        public static final double kLockPIDLeftiLimit = 0.3;

        public static final double kLockPIDRightkP = 0.1;
        public static final double kLockPIDRightkI = 0.1;
        public static final double kLockPIDRightkD = 0.01;
        public static final double kLockPIDRightiLimit = 0.3;
    }
    
    public final class AutoConstants {
        // feedforward shit
        public static final int kRamseteB = 0;
        public static final int kRamseteZeta = 0;
        public static final int kMaxSpeedMetersPerSecond = 0;
        public static final int kMaxAccelerationMetersPerSecondSquared = 0;
    }

    public final class LuffyConstants {
        public static final double strechSpeedScaler = 0.5;
        public static final int motorID = 0;
    }
    

    public final class OtotakeHirotadaConstants {
        public static final int compressorID = 0;
        public static final int DoublePCM1ForwardChannel = 0;
        public static final int DoublePCM1ReverseChannel = 1;
        public static final int DoublePCM2ForwardChannel = 2;
        public static final int DoublePCM2ReverseChannel = 3;
    }

    public final class GrabberConstants {
        public static final int compressorID = 0;
        public static final int DoublePCM1ForwardChannel = 0;
        public static final int DoublePCM1ReverseChannel = 1;
        public static final int DoublePCM2ForwardChannel = 2;
        public static final int DoublePCM2ReverseChannel = 3;
    }

    public final class ElevatorConstants {
        public static final int leftMotorID = 0;
        public static final int rightMotorID = 0;
        public static final double elevatorSpeedScaler = 0.5;
    }

    public final class OIConstants {
        public static final int driverController = 0;
        public static final int operatorController = 1;
        public static final int leftStick_X = 0;
        public static final int leftStick_Y = 1;
        public static final int trigger_L = 2;
        public static final int trigger_R = 3;
        public static final int rightStick_X = 4;
        public static final int rightStick_Y = 5;
        public static final int Btn_A = 1;
        public static final int Btn_B = 2;
        public static final int Btn_X = 3;
        public static final int Btn_Y = 4;
        public static final int Btn_LB = 5;
        public static final int Btn_RB = 6;
        public static final int Btn_LS = 9;  
        public static final int Btn_RS = 10;
    }
}