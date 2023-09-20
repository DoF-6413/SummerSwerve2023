// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
/* TODO: update imports
trajectory type (States that represent the pose, curvature, time, velocity, acceleration at that point.
Nested Class Summary)
(done) trajectory generator (creates trajectory from initial & final poses)
inside of ^ TrajectoryConfig​(double maxVelocityMetersPerSecond, double maxAccelerationMetersPerSecondSq)
swervecontrollercommand (runs trajectory)
*/

import java.util.List;

import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.pose.Pose;

public class autoNav extends CommandBase {
  public final Drive driveTrainSubusystem;
  public final Pose poseSubsystem;
  public final Pose2d startPose2d;
  public final Pose2d destinationPose2d;
  /** Creates a new autoNav. */
  
  public autoNav(Drive drive, Pose pose, Pose2d startPose, Pose2d destinationPose){
    driveTrainSubusystem = drive;
  poseSubsystem = pose;
  startPose2d = startPose;
  destinationPose2d = destinationPose;
  addRequirements(driveTrainSubusystem, poseSubsystem);
}
// When the command is initially scheduled we create a trajectory.
@Override
public void initialize() {
  /*
  TODO: define the following arguments to pass in to traj Generator
  Pose2d start = get initial pose,
  List<Translation2d> interiorWaypoints,
  Pose2d end = destination pose,
  TrajectoryConfig config = limit of velocity and acceleration,
  */
  poseSubsystem.getCurrentPose2d(); 
  
} 

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
    Trajectory generateTrajectory(
      Pose2d startPose = poseSubsystem.getCurrentPose2d(), 
      List<Translation2d> interiorWaypoints,
      destinationPose
      new TrajectoryConfig(0, 0));
    //use swervecontrollercommand to run runnableTrajectory    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // finish cmd once current pose == destination pose
    return false;
  }
}
