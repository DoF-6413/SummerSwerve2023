// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pose;

import org.littletonrobotics.junction.networktables.LoggedDashboardInput;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.subsystems.vision.Vision;

/** Add your docs here. */
public class Pose extends SubsystemBase {
private Field2d field2d;
private Drive drive;
private Gyro gyro;
private Vision vision;
private PhotonPoseEstimator p 
    public Pose(Drive drive, Gyro gyro, Vision vision, SwerveDriveKinematics kinematics) {
        field2d = new Field2d();
        SmartDashboard.putData(field2d);
        this.drive = drive;
        this.gyro = gyro;
        this.vision = vision;

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, this.gyro.getYaw(), this.drive.getSwerveModulePositions(), )
        
    }
}
