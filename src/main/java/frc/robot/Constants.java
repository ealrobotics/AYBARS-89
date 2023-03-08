// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class CANIDConstants {
    public static final int pcmID = 3;
    public static final int drivebaseLeftLeadMotorID = 13;
    public static final int drivebaseLeftFollowMotorID = 14;
    public static final int drivebaseRightLeadMotorID = 11;
    public static final int drivebaseRightFollowMotorID = 12;
    public static final int elevatorPivotMotorID = 30;
    public static final int elevatorLeadMotorID = 32;
    public static final int elevatorFollowMotorID = 31;
  }

  public static final class DriveConstants {
    public static final boolean kLeftLeadMotorInverted = true;
    public static final boolean kLeftFollowMotorInverted = true;
    public static final boolean kRightLeadMotorInverted = false;
    public static final boolean kRightFollowMotorInverted = false;

    public static final double kP = 4.416;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kLeftP = 4.416;
    public static final double kLeftI = 0;
    public static final double kLeftD = 0;

    public static final double kRightP = 4.416;
    public static final double kRightI = 0;
    public static final double kRightD = 0;

    public static final double kS = 0.88922;
    public static final double kV = 3.0889;

    // public static final int[] kLeftEncoderPorts = new int[] { 9, 8 }; --E4T
    public static final int[] kLeftEncoderPorts = new int[] { 0, 1 };
    public static final boolean kLeftEncoderReversed = true;

    // public static final int[] kRightEncoderPorts = new int[] { 7, 6 }; --E4T
    public static final int[] kRightEncoderPorts = new int[] { 2, 3 };
    public static final boolean kRightEncoderReversed = false;

    // public static final int kEncoderCPR = 360; --E4T
    public static final int kEncoderCPR = 512;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(6);
    public static final double kEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
    public static final double kTrackWidthMeters = 0.6545;

    public static final boolean kGyroReversed = false;

    public static final double kMaxSpeedPercentage = 0.5;
    public static final double kBoostedMaxSpeedPercentage = 1.0;

    public static final double kMaxSpeed = 3.0; // meters per second
    public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
    public static final double kSpeedRateLimit = 6;
    public static final double kRotRateLimit = 6;

  }

  public static final class AutoConstants {
    public static final PathConstraints kConstraints = new PathConstraints(3, 1);
  }

  public static final class ElevatorConstants {
    public static final boolean kInverted = false;
    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kMin = -0.3;
    public static final double kMax = 0.3;
    public static final double kRampRate = 0.5;
  }

  public static final class PivotConstants {
    public static final boolean kInverted = true;
    public static final double kP = 0.6;
    public static final double kI = 0;
    public static final double kD = 0.075;
    public static final double kMin = -1.0;
    public static final double kMax = 1.0;
    public static final double kRampRate = 0.5;
  }

  public static final class GripperConstants {
    public static final int kGripperForward = 1;
    public static final int kGripperReverse = 2;
    public static final Value kGripperDefaultState = Value.kForward;
  }

  public static final class Setpoints {
    public static final double kElevatorHigh = 80;
    public static final double kPivotHigh = 300;
    public static final double kElevatorMid = 50;
    public static final double kPivotMid = 200;
    public static final double kElevatorLow = 10;
    public static final double kPivotLow = 50;
  }

  public static Alliance alliance;
}
