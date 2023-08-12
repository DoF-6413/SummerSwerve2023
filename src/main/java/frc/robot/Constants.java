// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.utils.units;
import frc.robot.utils.units.*;

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

  public static Mode getMode(){
    if(RobotBase.isReal()){
      return Mode.REAL;
    } else if(RobotBase.isSimulation()){
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
    public static final double WHEEL_RADIUS_METERS = units.inchesToMeters(1.5);

    // PID Tuning for Drive Motors on Swerve Drive
    public static final double driveKp = 0.0;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.2;
    public static final double driveKv = 0.2;

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
    public static final double trackWidthX = 21.5; //21 from center of wheel to center of other wheel but 22 from encoder to encoder 
    public static final double trackWidthY = 21.5; //21 from center of wheel to center of other wheel but 22 from encoder to encoder

    private static final boolean isBrakemode = false;

    public static boolean ischaracterizing = false;

  }
  // drive constanst
  // wheel radius in meters

  public static class VisionConstants {
    //TODO: Update Transform to Correct Values and Make Universal for Multiple Cameras
    public static final Transform3d cameraOnRobot = new Transform3d( 
      new Translation3d(0.06, 0.18,-1.1176),
      new Rotation3d(0,0,2.95));
      
  }
  // how many swerve modules we have
  private double characterizationVolts = 0.0;

}
