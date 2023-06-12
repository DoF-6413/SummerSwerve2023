// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Vision extends SubsystemBase{

    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private final VisionIO visionIO;

    public Vision (VisionIO io) {
        visionIO = io;
    }

    public void periodic() { 
        visionIO.updateInputs(inputs);
        Logger.getInstance().processInputs("Vision", inputs);
    }

    public PhotonPipelineResult getResults(){
        return inputs.photonPipelineResult;
    }


    public double getTargetX() {
        return inputs.targetX;
    }

    public double getTargetY() {
        return inputs.targetY;
    }

    public double getTargetZ() {
        return inputs.targetZ;
    }

    public double getTargetYaw() {
        return inputs.targetYaw;
    }

    public double getTargetPitch() {
        return inputs.targetPitch;
    }

    public double getTargetArea() {
        return inputs.targetArea;
    }

    public boolean doesHaveTargets() {
        return inputs.hasTargets;
    }

    public int getBestFiducialID() {
        return inputs.bestFiducialID;
    }


}
