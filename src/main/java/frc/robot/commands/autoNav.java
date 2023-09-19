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
drive
pose */

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class autoNav extends CommandBase {
  /** Creates a new autoNav. */
  public autoNav(
    // TODO: uncomment arguments drive, pose, destination
  )
  {
    // TODO: Use addRequirements(drive & pose) here to declare subsystem dependencies
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

    // TODO: finish imports for final Trajectory runnableTrajectory = TrajectoryGenerator(new Pose2d(), new List<Translation2d>, new Pose2d(), new TrajectoryConfig(0, 0)) {}
  } 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
