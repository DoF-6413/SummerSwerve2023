// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gyro.Gyro;

public class QuickAuto extends CommandBase {
  public final Drive drivetrainSubsystem;
  public final Gyro gyroSubsystem;
  public Timer m_timer; 
  double m_time;

  /** Creates a new QuickAuto. */
  public QuickAuto(Drive drive, Gyro gyro, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrainSubsystem = drive;
    gyroSubsystem = gyro;
    m_time = time;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer = new Timer();
    m_timer.start();
    gyroSubsystem.updateHeading();
    new DefaultDriveCommand(drivetrainSubsystem, gyroSubsystem, ()->0.5, ()-> 0, ()-> 0).schedule();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("running");
    // new DefaultDriveCommand(drivetrainSubsystem, gyroSubsystem, ()->0.5, ()-> 0, ()-> 0).schedule();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("end running");
    new DefaultDriveCommand(drivetrainSubsystem, gyroSubsystem, ()-> 0, ()-> 0, ()-> 0).schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() >= m_time;
  }
}
