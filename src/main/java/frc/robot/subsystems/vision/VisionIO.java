// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public interface VisionIO {
    
    @AutoLog
    public static class VisionIOInputs{
        public PhotonPipelineResult photonPipelineResult = new PhotonPipelineResult();
        public boolean hasTargets = false;
        public PhotonTrackedTarget target = null;
        public int bestFiducialID = 0;
        public Transform3d bestCamToTarg = new Transform3d();
        public double targetX = 0.0;
        public double targetY = 0.0;
        public double targetZ = 0.0;
        public double targetYaw = 0.0;
        public double targetPitch = 0.0;
        public double targetArea = 0.0;

    
    }

    public default void updateInputs(VisionIOInputs inputs){

    }

}
