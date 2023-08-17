// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.gyro.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

public class DefaultDriveCommand extends CommandBase {
  private final Drive drivetrainSubsystem;
  private final Gyro gyroSubsystem;
  private final DoubleSupplier translationalXSupplier, translationalYSupplier, rotationSupplier;


  public DefaultDriveCommand(
    Drive drive, 
    Gyro gyro,
    DoubleSupplier translationalXSupplier,
    DoubleSupplier translationalYSupplier,
    DoubleSupplier rotationSupplier) {
      drivetrainSubsystem = drive;
      gyroSubsystem = gyro;
      this.translationalXSupplier = translationalXSupplier;
      this.translationalYSupplier = translationalYSupplier;
      this.rotationSupplier = rotationSupplier;
    addRequirements(drivetrainSubsystem, gyroSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrainSubsystem.runVelocity(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        translationalXSupplier.getAsDouble(), 
        translationalYSupplier.getAsDouble(), 
        rotationSupplier.getAsDouble(), 
        gyroSubsystem.getYaw())
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.runVelocity(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        0, 
        0, 
        0, 
        gyroSubsystem.getYaw())
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
