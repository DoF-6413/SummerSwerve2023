// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.EndEffectorConstants;

/** Add your docs here. */
public class EndEffectorIOSparkMax implements EndEffectorIO{
    private final CANSparkMax endEffectorMotor;
    private final RelativeEncoder endEffectorEncoder;

    public EndEffectorIOSparkMax(){
        endEffectorMotor = new CANSparkMax(EndEffectorConstants.endEffectorCANID, MotorType.kBrushless);
        endEffectorEncoder = endEffectorMotor.getEncoder();
    }

    public void updateInputs(EndEffectorIOInputs inputs){
        inputs.endEffectorPositionRad = Units.rotationsToRadians(endEffectorEncoder.getPosition()) / EndEffectorConstants.gearRatio;
        inputs.endEffectorVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(endEffectorEncoder.getVelocity()) / EndEffectorConstants.gearRatio;
        inputs.endEffectorAppliedVolts = endEffectorMotor.getAppliedOutput() * endEffectorMotor.getBusVoltage();
        inputs.endEffectorCurrentAmps = new double[] {endEffectorMotor.getOutputCurrent()};
    }

    public void setVoltage(double voltage){
        endEffectorMotor.setVoltage(voltage);
    }
}