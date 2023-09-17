// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.WristNeoConstants;

/** Add your docs here. */
public class WristIONeo implements WristIO{

private final CANSparkMax WristMotor;
private final RelativeEncoder wristEncoder;

public WristIONeo(){


WristMotor = new CANSparkMax(9,MotorType.kBrushless);
wristEncoder = WristMotor.getEncoder();
WristMotor.setIdleMode(IdleMode.kBrake);
}  
public void setWristSpeed(Double speed) {
    WristMotor.setVoltage(speed);

}

public void updateInputs(WristIOInputs inputs){
    inputs.turnAppliedVolts = WristMotor.getBusVoltage();
    inputs.turnPositionRad = Units.rotationsToRadians(wristEncoder.getPosition()) / WristNeoConstants.gearRatio;
    inputs.WristTempCelcius = WristMotor.getMotorTemperature();
    inputs.turnVelocityRadPerSec =  Units.rotationsPerMinuteToRadiansPerSecond(wristEncoder.getVelocity()) / WristNeoConstants.gearRatio;
}



}
