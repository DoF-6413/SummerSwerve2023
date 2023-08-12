// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.pose.Pose;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

public class FollowTrajectory extends CommandBase  {
public final Pose poseSubsystem;
public final Drive drivetrainSubsystem;
public final boolean isFirstPath;
public final PathPlannerTrajectory trajectory;
  /** Creates a new FollowTrajectory. */
  public FollowTrajectory(Drive drive, Pose pose, PathPlannerTrajectory traj, boolean firstPath) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrainSubsystem = drive;
    poseSubsystem = pose;
    trajectory = traj;
    isFirstPath = firstPath;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(isFirstPath) {
      poseSubsystem.resetPose(trajectory.getInitialHolonomicPose());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
new PPSwerveControllerCommand(
  trajectory, 
  poseSubsystem::getCurrentPose2d, 
  new PIDController(0, 0, 0), 
  new PIDController(0, 0, 0), 
  new PIDController(0, 0, 0), 
  drivetrainSubsystem::runVelocity,  
  true, // 
  drivetrainSubsystem,
  poseSubsystem);
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
