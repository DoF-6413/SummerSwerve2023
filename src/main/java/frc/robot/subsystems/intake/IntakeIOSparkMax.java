// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeMotor;

/** Add your docs here. */
public class IntakeIOSparkMax implements IntakeIO {
  private final CANSparkMax leftIntakeMotor;
  private final CANSparkMax rightIntakeMotor;

    public IntakeIOSparkMax() {
      leftIntakeMotor = new CANSparkMax(IntakeMotor.Left.CAN_ID, MotorType.kBrushless);
      rightIntakeMotor = new CANSparkMax(IntakeMotor.Right.CAN_ID, MotorType.kBrushless);
      leftIntakeMotor.setIdleMode(IdleMode.kBrake);
      rightIntakeMotor.setIdleMode(IdleMode.kBrake);
      leftIntakeMotor.setInverted(IntakeConstants.leftMotorInverted);
      rightIntakeMotor.setInverted(IntakeConstants.rightMotorInverted);
    }
    
    public void updateInputs(IntakeIOInputs inputs) {
      inputs.intakePositionRad = 
        Units.rotationsToRadians(leftIntakeMotor.getEncoder().getPosition()) / IntakeConstants.gearRatio;
      inputs.intakeVelocityRadPerSec = 
        Units.rotationsPerMinuteToRadiansPerSecond(leftIntakeMotor.getEncoder().getVelocity()) / IntakeConstants.gearRatio;
      inputs.intakeAppliedVolts = leftIntakeMotor.getAppliedOutput() * leftIntakeMotor.getBusVoltage(); 
      inputs.intakeCurrentAmps = new double[] {leftIntakeMotor.getOutputCurrent()};
      inputs.intakeTempCelcius = new double[] {leftIntakeMotor.getMotorTemperature()};
    }

    public void setIntakeVoltage(double volts) {
      leftIntakeMotor.setVoltage(volts);
    }
}
