// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pose;

import java.io.IOException;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.fieldconstants;
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
    public PhotonPipelineResult photonPipelineResult;
    public double resultsTimestamp;
    
    private double previousPipelineTimestamp = 0;

    public Pose(Drive drive, Gyro gyro, Vision vision, SwerveDriveKinematics kinematics) {
        field2d = new Field2d();
        SmartDashboard.putData(field2d);
        this.drive = drive;
        this.gyro = gyro;
        this.vision = vision;

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyro.getYaw(), this.drive.getSwerveModulePositions(),
            new Pose2d(new Translation2d(),new Rotation2d()));
    }

    @Override
    public void periodic() {

        //TODO: Split into different functions/ put into respective subsysems (leave neccessary things here)
        //TODO: Make ALL Smartdashboard -> "logged" value
        field2d.setRobotPose(getCurrentPose2d());
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), gyro.getYaw(), drive.getSwerveModulePositions());
        
        photonPipelineResult = vision.getResults();
        resultsTimestamp = photonPipelineResult.getTimestampSeconds();

        SmartDashboard.putNumber("photonTime", photonPipelineResult.getTimestampSeconds());
        Logger.getInstance().recordOutput("TimeStampSec", photonPipelineResult.getTimestampSeconds());
        SmartDashboard.putNumber("FPGA TIme", Timer.getFPGATimestamp());
        Logger.getInstance().recordOutput("FPGA TIme", Timer.getFPGATimestamp());
        Logger.getInstance().recordOutput("CurrentPose2d",poseEstimator.getEstimatedPosition());
        Logger.getInstance().recordOutput("hastarget", vision.doesHaveTargets());        
        if (resultsTimestamp != previousPipelineTimestamp && vision.doesHaveTargets()) {
            previousPipelineTimestamp = resultsTimestamp;
            var target = photonPipelineResult.getBestTarget();
            var fiducialid = target.getFiducialId();
            if (target.getPoseAmbiguity() >= 0.2 && fiducialid >= 0 && fiducialid < 9) {
                
                AprilTagFieldLayout atfl;
                try {
                    atfl = new AprilTagFieldLayout(AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField().getTags(),
                    fieldconstants.fildlength,
                    fieldconstants.fildwidth);
                    Pose3d targetPose = atfl.getTagPose(fiducialid).get();
                    Logger.getInstance().recordOutput("targetPose", targetPose);
                    Transform3d camToTarget = target.getBestCameraToTarget();
                    Pose3d camPose = targetPose.transformBy(camToTarget);
                    
                    // Make Universal for Multiple Cameras
                    Pose3d visionMeasurement = camPose.transformBy(VisionConstants.cameraOnRobot);
                    SmartDashboard.putString("visionmeasure", visionMeasurement.toPose2d().toString());
                    Logger.getInstance().recordOutput("visionmeasure", visionMeasurement.toPose2d().toString());
                    SmartDashboard.putNumber("ResultTimeStamp", resultsTimestamp);
                    Logger.getInstance().recordOutput("ResultTimeStamp", resultsTimestamp);
                    poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(),
                    Timer.getFPGATimestamp(),
                    visionMeasurementStdDevs);
                    
                    
                    
                } catch (IOException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            }
        }
    }

    public Pose2d getCurrentPose2d() {
        return poseEstimator.getEstimatedPosition();
        //this
    }

    public void resetPose(Pose2d currentPose2d) {
        poseEstimator.resetPosition(gyro.getYaw(), drive.getSwerveModulePositions(), currentPose2d);
    }

    public void setPose2d() {
        if (Constants.getMode() == Mode.SIM) {
            field2d.setRobotPose(poseEstimator.getEstimatedPosition());
        }
    }

    public Pose2d getInitialPose() {
        return null;
    }
}
