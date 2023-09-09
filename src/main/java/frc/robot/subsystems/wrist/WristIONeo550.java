// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.WristConstants;

/** Add your docs here. */
public class WristIONeo550 implements WristIO{

private final CANSparkMax WristMotor;
private final RelativeEncoder wristEncoder;

public WristIONeo550(){


WristMotor = new CANSparkMax(2,MotorType.kBrushless);
wristEncoder = WristMotor.getEncoder();
}  
public void setWristSpeed(int speed) {
    WristMotor.setVoltage(speed);

}

public void updateInputs(WristIOInputs inputs){
    inputs.turnAppliedVolts = WristMotor.getBusVoltage();
    inputs.turnPositionRad = Units.rotationsToRadians(wristEncoder.getPosition()) / WristConstants.gearRatio;
    inputs.WristTempCelcius = WristMotor.getMotorTemperature();
    inputs.turnVelocityRadPerSec =  Units.rotationsPerMinuteToRadiansPerSecond(wristEncoder.getVelocity()) / WristConstants.gearRatio;
}



}
