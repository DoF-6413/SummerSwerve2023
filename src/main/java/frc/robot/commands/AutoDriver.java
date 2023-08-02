// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.subsystems.pose.Pose;


public class AutoDriver extends CommandBase {
  private final Drive drivetrainSubsystem;
  private final Gyro gyroSubsystem;
  private final PathPlannerTrajectory pathGroup;
  private final SwerveAutoBuilder autoBuilder;
  private final Pose pose;
  private final boolean isFirstPath;
  
  /** Creates a new AutoDriver. */
  public AutoDriver(Drive drive, 
  Gyro gyro, Pose pose, PathPlannerTrajectory trajectory, boolean firstPath) {
    //Everything here 
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("marker1", new PrintCommand("Passed marker 1"));
    //To here should be passed in as an argument to autoDriver
    drivetrainSubsystem = drive;
    gyroSubsystem = gyro;
    this.pose = pose;
    pathGroup = trajectory;
    isFirstPath = firstPath;
    addRequirements(drivetrainSubsystem, gyroSubsystem);
    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
     autoBuilder = new SwerveAutoBuilder(
        pose::getCurrentPose2d, // Pose2d supplier
        pose::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
        drive.swerveKinematics, // SwerveDriveKinematics
        new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        drive::useModuleStates, // Module states consumer used to output to the drive subsystem
        eventMap,
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        drive // The drive subsystem. Used to properly set the requirements of path following commands
    );
    
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Assuming this method is part of a drivetrain subsystem that provides the necessary methods

         // Reset odometry for the first path you run during auto
        //  if(isFirstPath){
        //      pose.reset(pathGroup.getInitialHolonomicPose());
        //  }
       new PPSwerveControllerCommand(
           pathGroup, 
           pose::getCurrentPose2d, // Pose supplier
           drivetrainSubsystem.swerveKinematics, // SwerveDriveKinematics
           new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
           new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           drivetrainSubsystem::useModuleStates, // Module states consumer
           true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
           drivetrainSubsystem, gyroSubsystem // Requires this drive subsystem
       );
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Takes autoBuilder in the class and runs with the pathGroup
    System.out.println("AutoDriver Running GG");
    autoBuilder.fullAuto(pathGroup);
    }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}