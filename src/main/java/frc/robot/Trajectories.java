package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

<<<<<<< HEAD
public class Trajectories {
 public static final PathPlannerTrajectory test = PathPlanner.loadPath("FullAuto", new PathConstraints(4, 3));
=======
public class Trajectories implements Command {
public static final PathPlannerTrajectory test = PathPlanner.loadPath("FullAuto", new PathConstraints(4, 3));

@Override
public Set<Subsystem> getRequirements() {
    // TODO Auto-generated method stub
    return null;
}
>>>>>>> 28712b4a84896d92a03e40b6d28ca01f23506140
}