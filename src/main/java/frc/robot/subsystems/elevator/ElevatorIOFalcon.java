// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorMotor;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorIOFalcon implements ElevatorIO {
  /** Creates a new ElevatorIOFalcon. */
private final WPI_TalonFX leftElevatorMotor;
private final WPI_TalonFX rightElevatorMotor;
private final DigitalInput leftLimitSwitch;
private final DigitalInput rightLimitSwitch;

  public ElevatorIOFalcon() {
    leftElevatorMotor = new WPI_TalonFX(ElevatorMotor.Left.CAN_ID);
    rightElevatorMotor = new WPI_TalonFX(ElevatorMotor.Right.CAN_ID);
    leftElevatorMotor.setNeutralMode(NeutralMode.Brake);
    rightElevatorMotor.setNeutralMode(NeutralMode.Brake);
    leftElevatorMotor.setInverted(ElevatorConstants.leftMotorInverted);
    rightElevatorMotor.setInverted(ElevatorConstants.rightMotorInverted);
    StatorCurrentLimitConfiguration currentLimitConfig = new StatorCurrentLimitConfiguration(
      ElevatorConstants.kIsElevatorCurrentLimitEnabled,
      ElevatorConstants.kElevatorContinuousCurrent,
      ElevatorConstants.kElevatorPeakCurrent,
      ElevatorConstants.kElevatorMaxTimeAtPeak);
      rightElevatorMotor.follow(leftElevatorMotor);
      leftElevatorMotor.configStatorCurrentLimit(currentLimitConfig);
      rightElevatorMotor.configStatorCurrentLimit(currentLimitConfig);
    
    leftLimitSwitch = new DigitalInput(0);
    rightLimitSwitch = new DigitalInput(1);
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.elevatorPositionRad = 
      Units.rotationsToRadians(leftElevatorMotor.getSelectedSensorPosition()) / ElevatorConstants.gearRatio;
    inputs.elevatorVelocityRadPerSec = 
      Units.rotationsPerMinuteToRadiansPerSecond(leftElevatorMotor.getSelectedSensorVelocity()) / ElevatorConstants.gearRatio;
    inputs.elevatorAppliedVolts = leftElevatorMotor.getMotorOutputVoltage() * leftElevatorMotor.getBusVoltage();
    inputs.elevatorCurrentAmps = new double[] {leftElevatorMotor.getStatorCurrent()};
    inputs.elevatorTempCelcius = new double[] {leftElevatorMotor.getTemperature()};
    
  }

  @Override
  public void setVoltageSpeed(double volts) {
    leftElevatorMotor.setVoltage(volts);
  }
}
