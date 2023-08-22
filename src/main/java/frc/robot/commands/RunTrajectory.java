// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.pose.Pose;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunTrajectory extends SequentialCommandGroup {
  /** Creates a new jhoncena. */
  public RunTrajectory(Drive drive, Pose pose, PathPlannerTrajectory traj, boolean firstPath) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    new InstantCommand(() -> {
 
      if(firstPath) {
        
        pose.resetPose(traj.getInitialHolonomicPose());
      }

    }),

    new InstantCommand(() -> 


      new PPSwerveControllerCommand(
        traj, 
        pose::getCurrentPose2d, 
        new PIDController(3, 0, 0), 
        new PIDController(3, 0, 0), 
        new PIDController(3, 0, 0), 
        drive::runVelocity,  
        true, 
        drive,
        pose).schedule()
    )

    );
  }
}
