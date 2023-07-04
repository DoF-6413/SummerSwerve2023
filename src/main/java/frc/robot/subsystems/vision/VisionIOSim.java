// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;

/** Add your docs here. */
public class VisionIOSim implements VisionIO {
    
    private static PhotonCamera camera = new PhotonCamera("FrontCamera");
    SimVisionSystem simVision =
            new SimVisionSystem(
                    "photonvision",
                    VisionConstants.camDiagFOV,
                    new Transform3d(
                            new Translation3d(0, 0, VisionConstants.camHeightOffGround), new Rotation3d(0, VisionConstants.camPitch, VisionConstants.camYaw)),
                            VisionConstants.maxLEDRange,
                            VisionConstants.camResolutionWidth,
                            VisionConstants.camResolutionHeight,
                            VisionConstants.minTargetArea); 

    public VisionIOSim() {
          
        // double[] aprilTag1 = {15.513558,1.071626,0.462788};
        // double[] aprilTag2 = {15.513558, 2.748026,0.462788};
        // double[] aprilTag3 = {15.513558,4.424426,0.462788};
        // double[] aprilTag4 = {16.178784,6.749796,0.695452};
        // double[] aprilTag5 = {0.36195,6.749796,0.695452};
        // double[] aprilTag6 = {1.02743,4.424426,0.462788};
        // double[] aprilTag7 = {1.02743,2.748026,0.462788};
        // double[] aprilTag8 = {1.02743,1.071626,0.4627288};

        SimVisionTarget aprilTag1 = new SimVisionTarget(
        new Pose3d(15.513558,1.071626,0.462788, new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0))), 0.15, 0.15, 1);
        SimVisionTarget aprilTag2 = new SimVisionTarget(
        new Pose3d(15.513558, 2.748026,0.462788, new Rotation3d(new Quaternion(0.0, 0.0,0.0, 1.0))), 0.15, 0.15, 2);
        SimVisionTarget aprilTag3 = new SimVisionTarget(
        new Pose3d(15.513558,4.424426,0.462788, new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0))), 0.15, 0.15, 3);
        SimVisionTarget aprilTag4 = new SimVisionTarget(
        new Pose3d(16.178784,6.749796,0.695452, new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0))), 0.15, 0.15, 4);
        SimVisionTarget aprilTag5 = new SimVisionTarget(
        new Pose3d(0.36195,6.749796,0.695452, new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))), 0.15, 0.15, 5);
        SimVisionTarget aprilTag6 = new SimVisionTarget(
        new Pose3d(1.02743,4.424426,0.462788, new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))), 0.15, 0.15, 6);
        SimVisionTarget aprilTag7 = new SimVisionTarget(
        new Pose3d(1.02743,2.748026,0.462788, new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))), 0.15, 0.15, 7);
        SimVisionTarget aprilTag8 = new SimVisionTarget(
        new Pose3d(1.02743,1.071626,0.4627288, new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))), 0.15, 0.15, 8);
        // SmartDashboard.putNumberArray("apriltag1",aprilTag1);
        // SmartDashboard.putNumberArray("apriltag2",aprilTag2);
        // SmartDashboard.putNumberArray("apriltag3",aprilTag3);
        // SmartDashboard.putNumberArray("apriltag4",aprilTag4);
        // SmartDashboard.putNumberArray("apriltag5",aprilTag5);
        // SmartDashboard.putNumberArray("apriltag6",aprilTag6);
        // SmartDashboard.putNumberArray("apriltag7",aprilTag7);
        // SmartDashboard.putNumberArray("apriltag8",aprilTag8);

        simVision.addSimVisionTarget(aprilTag1);
        simVision.addSimVisionTarget(aprilTag2);
        simVision.addSimVisionTarget(aprilTag3);
        simVision.addSimVisionTarget(aprilTag4);
        simVision.addSimVisionTarget(aprilTag5);
        simVision.addSimVisionTarget(aprilTag6);
        simVision.addSimVisionTarget(aprilTag7);
        simVision.addSimVisionTarget(aprilTag8);

    }

    public void updateInputs(VisionIOInputs inputs) {
        // inputs.photonPipelineResult = camera.getLatestResult();
        // inputs.hasTargets = simVision.;
        // if (inputs.hasTargets == true) {
        //     inputs.target = inputs.photonPipelineResult.getBestTarget();
        //     inputs.bestFiducialID = inputs.target.getFiducialId();
        //     inputs.bestCamToTarg = inputs.target.getBestCameraToTarget();
        //     inputs.targetX = inputs.bestCamToTarg.getX();
        //     inputs.targetY = inputs.bestCamToTarg.getY();
        //     inputs.targetZ = inputs.bestCamToTarg.getZ();
        //     inputs.targetYaw = inputs.target.getYaw();
        //     inputs.targetPitch = inputs.target.getPitch();
        //     inputs.targetArea = inputs.target.getArea();

        // }
        

        inputs.simVision = simVision;
    }

}
