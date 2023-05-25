// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.CAN;

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
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class DrivetrainConstants {

    public enum DriveMotor {
      frontLeft(0), // TODO:update value
      frontRight(1), // TODO:update value
      backLeft(2), // TODO:update value
      backRight(3); // TODO:update value

      public final int CAN_ID;

      DriveMotor(int value) {
        CAN_ID = value;
      }

    }

    public enum TurnMotor {
      frontLeft(4), // TODO:update value
      frontRight(5), // TODO:update value
      backLeft(6), // TODO:update value
      backRight(7); // TODO:update value

      public final int CAN_ID;

      TurnMotor(int value) {
        CAN_ID = value;
      }
    }

    // Wheel Facts
    public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(3.0);

    // PID Tuning for Drive Motors on Swerve Drive
    public static final double driveKp = 0.0;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.2;
    public static final double driveKv = 0.2;

    // PID Tuning for Turn Motors on Swerve Drive
    public static final double turnKp = 0.0;
    public static final double turnKd = 0.0;
    public static final double turnKs = 0.2;
    public static final double turnKv = 0.2;

    public static final int CANConfigTimeout = 500;
    public static final double loopPeriodSecs = 0.02;
  }
  // drive constanst
  // wheel radius in meters

  // how many swerve modules we have
  private double characterizationVolts = 0.0;
  // TODO:Change non real values to real one's(I use "placeholders")

}
