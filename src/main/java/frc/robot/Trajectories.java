package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class Trajectories {
 public static final PathPlannerTrajectory test = PathPlanner.loadPath("FullAuto", new PathConstraints(4, 3));
}