// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static Mode getMode() {
    if (RobotBase.isReal()) {
      return Mode.REAL;
    } else if (RobotBase.isSimulation()) {
      return Mode.SIM;
    } else {
      return Mode.REPLAY;
    }
  }

  public static Alliance getAlliance() {
    return DriverStation.getAlliance();
  }

  public static final int CANConfigTimeout = 500;
  public static final double loopPeriodSecs = 0.02;

  public static class OperatorConstants {
    public static final int DriveController = 0;
  }

  // TODO:Change non real values to real one's(I use "placeholders")
  public static class DrivetrainConstants {

    public static enum AbsoluteEncoder{
      FrontLeft(8),
      frontRight(11),
      BackLeft(5),
      BackRigth(2);

      public final int EncoderID;
      AbsoluteEncoder(int ID){
       EncoderID = ID;
      }
    }

    public static enum AbsoluteEncoderOffset{
      FrontLeft(-87.6269),
      frontRight( -89.7363),
      BackLeft(-356.0449),
      BackRigth(-177.6269);
     
      public final double offset;
      AbsoluteEncoderOffset(double value){
        offset = value;
      }
    }

    public static final double driveAfterEncoderReduction = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);

    public enum DriveMotor {
      frontLeft(10),
      frontRight(13),
      backLeft(7),
      backRight(4);

      public final int CAN_ID;

      

      DriveMotor(int value) {
        CAN_ID = value;
      }

    }

    public enum TurnMotor {
      frontLeft(9),
      frontRight(12),
      backLeft(6),
      backRight(3);

      public final int CAN_ID;

      TurnMotor(int value) {
        CAN_ID = value;
      }
    }

    // Wheel Facts
    public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.5);

    // PID Tuning for Drive Motors on Swerve Drive
    public static final double driveKp = 0;
    public static final double driveKd = 0;
    public static final double driveKs = 0.4;
    public static final double driveKv = 0.4;

    // PID Tuning for Turn Motors on Swerve Drive
    public static final double turnKp = 7.0;
    public static final double turnKd = 0.1;
    public static final double turnKs = 0.2;
    public static final double turnKv = 0.2;

    // Under this to switch to coast when disabling
    public static final double coastThresholdMetersPerSec = 0.05;
    // Under this speed for this length of time to switch to coast
    public static final double coastThresholdSec = 6.0;
    // Threshold to detect falls
    public static final double ledsFallenAngleDegrees = 60.0;

    public static final double maxLinearSpeed = 4.5; //meters/sec 

    public static final double trackWidthX = Units.inchesToMeters(21.5); //21 from center of wheel to center of other wheel but 22 from encoder to encoder 
    public static final double trackWidthY = Units.inchesToMeters(21.5); //21 from center of wheel to center of other wheel but 22 from encoder to encoder

    private static final boolean isBrakemode = false;

    public static boolean ischaracterizing = false;


  }
  // drive constanst
  // wheel radius in meters

  public static class VisionConstants {
    // TODO: Update Transform to Correct Values and Make Universal for Multiple
    // Cameras
    public static final Transform3d cameraOnRobot = new Transform3d(
        new Translation3d(0.06, 0.18, -1.1176),
        new Rotation3d(0, 0, 2.95));

  }

  // how many swerve modules we have
  private double characterizationVolts = 0.0;

  public static class fieldconstants {

    public static final double fildlength = 16.4592;
    public static final double fildwidth = 8.2296;
  }
public static class ElevatorConstants{
  public enum ElevatorMotor {
      Left(10),
      Right(13); 
    

      public final int CAN_ID;

      ElevatorMotor(int value) {
        CAN_ID = value;
      }

}
public final static double elevatorConversionFactor = 0.0;
public final static double gearRatio = 7;
public final static double falconTicks = 2048;
public final static boolean leftMotorInverted = true;
public final static boolean rightMotorInverted = false;
//Elevetor Current Limit
public static final boolean kIsElevatorCurrentLimitEnabled = true;
public static final double kElevatorContinuousCurrent = 40;
public static final double kElevatorPeakCurrent = 60;
public static final double kElevatorMaxTimeAtPeak = 5.0;
}
}