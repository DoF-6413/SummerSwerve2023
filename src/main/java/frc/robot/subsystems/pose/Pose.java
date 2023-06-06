// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pose;

import java.io.IOException;

import org.littletonrobotics.junction.networktables.LoggedDashboardInput;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.subsystems.gyro.GyroIOSim;
import frc.robot.subsystems.vision.Vision;
import edu.wpi.first.math.Vector;

/** Add your docs here. */
public class Pose extends SubsystemBase {
    /**
   * Standard deviations of model states. Increase these numbers to trust your
   * model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
   * meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);

  /**
   * Standard deviations of the vision measurements. Increase these numbers to
   * trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
   * radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(1.0, 1.0, 1.0);

private Field2d field2d;
private Drive drive;
private Gyro gyro;
private Vision vision;
private SwerveDrivePoseEstimator poseEstimator;
 
    public Pose(Drive drive, Gyro gyro, Vision vision, SwerveDriveKinematics kinematics) {
        field2d = new Field2d();
        SmartDashboard.putData(field2d);
        this.drive = drive;
        this.gyro = gyro;
        this.vision = vision;

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyro.getYaw(), this.drive.getSwerveModulePositions(), new Pose2d());
        
    } 

    @Override 
    public void periodic() {
        field2d.setRobotPose(getCurrentPose2d());
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), gyro.getYaw(), drive.getSwerveModulePositions());
        
    }


    public Pose2d getCurrentPose2d() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d currentPose2d) {
        poseEstimator.resetPosition(gyro.getYaw(), drive.getSwerveModulePositions(), currentPose2d);
    }

    public void setPose2d() {
        if(Constants.getMode() == Mode.SIM) {
        field2d.setRobotPose(poseEstimator.getEstimatedPosition());
        }
    }
}