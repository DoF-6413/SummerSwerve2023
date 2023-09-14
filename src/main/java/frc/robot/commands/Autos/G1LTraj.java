// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.pose.Pose;

/** Add your docs here. */
public class G1LTraj extends SequentialCommandGroup {

    PathPlannerTrajectory kG1LTraj = PathPlanner.loadPath("g1LTraj", new PathConstraints(1.2, 8));;


    public G1LTraj(){
        addCommands(

        new G1LTraj()
        );
    }
}
