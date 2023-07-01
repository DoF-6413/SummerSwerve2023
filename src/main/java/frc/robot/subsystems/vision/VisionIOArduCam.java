// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;

//https://github.dev/DoF-6413/chargedUp/blob/103-actualize-aux-controller/chargedUp/src/main/java/frc/robot/subsystems/VisionSubsystem.java
/** Add your docs here. */
public class VisionIOArduCam implements VisionIO{
    private static PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

    public VisionIOArduCam(){
    }
    
    public void updateInputs(VisionIOInputs inputs){
        inputs.photonPipelineResult = camera.getLatestResult();
        inputs.hasTargets = inputs.photonPipelineResult.hasTargets();
        if(inputs.hasTargets == true){
            inputs.target = inputs.photonPipelineResult.getBestTarget();
            inputs.bestFiducialID = inputs.target.getFiducialId();
            inputs.bestCamToTarg = inputs.target.getBestCameraToTarget();
            inputs.targetX = inputs.bestCamToTarg.getX();
            inputs.targetY = inputs.bestCamToTarg.getY();
            inputs.targetZ = inputs.bestCamToTarg.getZ();
            inputs.targetYaw = inputs.target.getYaw();
            inputs.targetPitch = inputs.target.getPitch();
            inputs.targetArea = inputs.target.getArea(); 
            
        }
    }
}
