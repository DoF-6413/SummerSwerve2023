package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Trajectories {
 public static final PathPlannerTrajectory test = PathPlanner.loadPath("FullAuto", new PathConstraints(4, 3));
}