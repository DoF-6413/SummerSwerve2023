// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.photonvision.SimVisionTarget;

/** Add your docs here. */
public class VisionIOSim implements VisionIO {

    public VisionIOSim() {
        //TODO: Update April Tag 1 Params and creat April Tags 2-8 OR Remove and Add Sim Camera 
        SimVisionTarget aprilTag1 = new SimVisionTarget(null, 0, 0, 0);
    }

    public void updateInputs(VisionIOInputs inputs) {
        inputs.photonPipelineResult = camera.getLatestResult();
        inputs.hasTargets = inputs.photonPipelineResult.hasTargets();
        if (inputs.hasTargets == true) {
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
