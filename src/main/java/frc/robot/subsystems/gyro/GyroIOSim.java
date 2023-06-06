// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gyro;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class GyroIOSim implements GyroIO{
    private SimDevice simGyro;
    private SimDouble simAngle = null;
    private SimDouble simRate = null;
    private SimDouble simPitch = null;
    private SimDouble simRoll = null;
    private SimDouble simDisplacementX = null;
    private SimDouble simDiplacementY = null;
    private SimDouble simDisplacementZ = null;


    public GyroIOSim(){
        simGyro = SimDevice.create("AHRS", SPI.Port.kMXP.value);
    if (simGyro != null){
        simAngle = simGyro.createDouble("Angle", Direction.kOutput, 0.0);
        simRate = simGyro.createDouble("Rate", Direction.kOutput,0.0);
        simPitch = simGyro.createDouble("Pitch", Direction.kOutput,0.0); 
        simRoll = simGyro.createDouble("Roll", Direction.kOutput, 0.0);
        simDisplacementX = simGyro.createDouble("DisplacementX", Direction.kOutput, 0.0);
        simDiplacementY = simGyro.createDouble("DisplacementY",Direction.kOutput, 0.0);
        simDisplacementZ = simGyro.createDouble("DisplacementZ",Direction.kOutput, 0.0);

    }
    }

    public void updateInputs(GyroIOInputs inputs){
        inputs.connected = true;
        inputs.rollPositionRad = Units.degreesToRadians(simRoll.get());
        inputs.pitchPositionRad = Units.degreesToRadians(simPitch.get()) ;
        inputs.yawPositionRad = Units.degreesToRadians(simAngle.get());
        inputs.rate = simRate.get();
        inputs.pitchVelocityRadPerSec = simDiplacementY.get();
        inputs.yawVelocityRadPerSec = simDisplacementX.get();
        inputs.rollVelocityRadPerSec = simDisplacementZ.get();
    }

    
}
