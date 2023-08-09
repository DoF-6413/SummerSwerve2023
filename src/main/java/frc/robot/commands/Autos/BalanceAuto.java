// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gyro.Gyro;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BalanceAuto extends CommandBase {
  public final Drive driveTrainSubsystem;
  public final Gyro gyroSubsystem;
  public Timer m_timer;
  double m_time; 
    
  /** Creates a new BalanceAuto. */
  public BalanceAuto(Drive drive, Gyro gyro, double time) {
    driveTrainSubsystem = drive;
    gyroSubsystem = gyro;
    m_time = time;
  }

    @Override
    public void initialize() {
      m_timer = new Timer();
      m_timer.start();
      gyroSubsystem.updateHeading();
      new DefaultDriveCommand(driveTrainSubsystem, gyroSubsystem, ()->0.5, ()-> 0, ()-> 0).schedule();
      // gyroSubsystem.getPitch();
      // gyroSubsystem.getYaw();
      // gyroSubsystem.getRoll(); 
      driveTrainSubsystem.setRaw(0, 0);
  }


                                                                                                                                           
    @Override
    public void execute() {
      if(gyroSubsystem.getRoll().getDegrees() > -10 && gyroSubsystem.getRoll().getDegrees() < 10) {
        driveTrainSubsystem.setRaw(0,0);
      }
      else if(gyroSubsystem.getRoll().getRadians() > 0){
        driveTrainSubsystem.setRaw(0.35, 0);
      } 
      else if (gyroSubsystem.getRoll().getRadians() < 0){
          driveTrainSubsystem.setRaw(-0.35, 0);
      } 
      else {
          driveTrainSubsystem.setRaw(0, 0);
      }
      
      /**
       * ToDo: define mount angle 
       * ToDo: create the balancer  
       * ToDo: define the dismount angle  
      */
      System.out.println("running balance");
    }

    @Override
    public void end(boolean interrupted) {
      System.out.println("end running");
      new DefaultDriveCommand(driveTrainSubsystem, gyroSubsystem, ()-> 0, ()-> 0, ()-> 0).schedule();
      driveTrainSubsystem.setRaw(0, 0);
    }

    @Override
    public boolean isFinished() {
    
    return m_timer.get() >= m_time;
  }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // addCommands();
    
}

