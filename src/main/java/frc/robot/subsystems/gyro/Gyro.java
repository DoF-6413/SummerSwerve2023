// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gyro;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Gyro extends SubsystemBase {
    public static GyroIO gyroIO;
    public static GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    public Gyro(GyroIO io) {
        gyroIO = io;
    }

    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        SmartDashboard.putNumber("gyroyaw" ,gyroInputs.yawPositionRad);
        SmartDashboard.putNumber("gyropitch", gyroInputs.pitchPositionRad);
        Logger.getInstance().processInputs("Gyro", gyroInputs);
    }

    /** Returns the current pitch (Y rotation). */
    public Rotation2d getPitch() {
        return new Rotation2d(gyroInputs.pitchPositionRad);
    }

    /** Returns the current roll (X rotation). */
    public Rotation2d getRoll() {
        return new Rotation2d(gyroInputs.rollPositionRad);
    }

    public Rotation2d getYaw() {
        return new Rotation2d(gyroInputs.yawPositionRad);
    }

    public void setYaw(double radians){
        gyroInputs.yawPositionRad = radians;
    }

    /** Returns the current yaw velocity (Z rotation) in radians per second. */
    public double getYawVelocity() {
        return gyroInputs.yawVelocityRadPerSec;
    }

    /** Returns the current pitch velocity (Y rotation) in radians per second. */
    public double getPitchVelocity() {
        return gyroInputs.pitchVelocityRadPerSec;
    }

    /** Returns the current roll velocity (X rotation) in radians per second. */
    public double getRollVelocity() {
        return gyroInputs.rollVelocityRadPerSec;
    }

    public boolean isConnected() {
        return gyroInputs.connected;
    }
}
