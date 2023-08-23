// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.QuickAuto;
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

  // private Command driveCommand;

    /** Creates a new BalanceAuto. */
  public BalanceAuto(Drive drive, Gyro gyro, double time) {
    driveTrainSubsystem = drive;
    gyroSubsystem = gyro;
    m_time = time;
    addRequirements(driveTrainSubsystem, gyroSubsystem);
    // driveCommand = new DefaultDriveCommand(driveTrainSubsystem, gyroSubsystem, ()->0, ()->0, ()->0);
  }

    @Override
    public void initialize() {
      m_timer = new Timer();
      m_timer.start();
      m_timer.reset();
      gyroSubsystem.updateHeading();
      // driveTrainSubsystem.removeDefaultCommand();
      // new DefaultDriveCommand(driveTrainSubsystem, gyroSubsystem, ()->0.5, ()-> 0, ()-> 0).schedule();
  }


                                                                                                                                           
    @Override
    public void execute() {
      if(gyroSubsystem.getRoll().getRadians() > 0.1){
        driveTrainSubsystem.setRaw(0.25, 0.0, 0.0);
        // new DefaultDriveCommand(driveTrainSubsystem, gyroSubsystem,()-> 0.2, ()-> 0.0, ()-> 0.0).schedule();;
      } 
      else if (gyroSubsystem.getRoll().getRadians() < -0.1) {
        driveTrainSubsystem.setRaw(-0.25, 0.0, 0.0);  
        // new DefaultDriveCommand(driveTrainSubsystem, gyroSubsystem,()-> -0.2, ()-> 0.0, ()-> 0.0).schedule();;
      } 
      else {
        driveTrainSubsystem.setRaw(0.0, 0.0, 0.0);    
        // new DefaultDriveCommand(driveTrainSubsystem, gyroSubsystem,()-> 0.0, ()-> 0.0, ()-> 0.0).schedule();;
      }
      
      /**
       * ToDo: 
      */
      System.out.println("running balance");
    } 

    @Override 
    public void end(boolean interrupted) {
      // driveTrainSubsystem.setRaw(0, 0, 0);  
      new DefaultDriveCommand(driveTrainSubsystem, gyroSubsystem,()-> 0.0, ()-> 0.0, ()-> 0.0);  
      System.out.println("end running");
    }

    @Override
    public boolean isFinished() {
    return m_timer.get() >= m_time;
    // return false;
  }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // addCommands();
    
}
