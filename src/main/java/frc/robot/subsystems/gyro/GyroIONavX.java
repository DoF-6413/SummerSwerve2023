// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gyro;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;

/** Add your docs here. */
public class GyroIONavX implements GyroIO {

    private final AHRS gyro;

    public GyroIONavX() {
        gyro = new AHRS(SPI.Port.kMXP, (byte) 200);
        gyro.calibrate();
        gyro.zeroYaw();
        gyro.reset();
        gyro.resetDisplacement();
    }

    public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = gyro.isConnected();
    inputs.rollPositionRad = Units.degreesToRadians(gyro.getRoll());
    inputs.pitchPositionRad = Units.degreesToRadians(gyro.getPitch()) ;
    inputs.yawPositionRad = Units.degreesToRadians(gyro.getYaw());
    inputs.pitchVelocityRadPerSec = gyro.getDisplacementY();
    inputs.yawVelocityRadPerSec = gyro.getDisplacementZ();
    inputs.rollVelocityRadPerSec = gyro.getDisplacementX();
    }

}