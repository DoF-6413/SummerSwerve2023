package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.DrivetrainConstants;

public class autoNavChooser {
    private int m_grid = 2;
    private int m_col = 0;
    Trajectory chosenTraj;
    TrajectoryConfig config;

    public autoNavChooser(int grid, int col) {
        m_grid = grid;
        m_col = col;
    }
    
}