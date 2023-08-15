// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.QuickAuto;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gyro.Gyro;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class DriveAndBalance extends SequentialCommandGroup {
  /** Creates a new DriveAndBalance. */
  public final Drive driveTrainSubsystem;
  public final Gyro gyroSubsystem; 
  // public final Timer m_timer;
  public DriveAndBalance(Drive drive, Gyro gyro) {
    driveTrainSubsystem = drive;
    gyroSubsystem = gyro;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new QuickAuto(drive, gyro, 3),
      new BalanceAuto(drive, gyro, 11)
    );
  }
}
