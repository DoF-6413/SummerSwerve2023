// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.DrivetrainConstants.DriveMotor;
import frc.robot.Constants.DrivetrainConstants.TurnMotor;

public class ModuleIOSparkMax implements moduleIO {
  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final CANCoder turnAbsoluteEncoder;

  private final double driveAfterEncoderReduction = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  private final double turnAfterEncoderReduction = 150.0 / 7.0;

  private final boolean isTurnMotorInverted = true;
  private final double absoluteEncoderOffset;
  private final int nancy;
  public ModuleIOSparkMax(int index) {
    System.out.println("[Init] Creating ModuleIOSparkMax " + Integer.toString(index));
      this.nancy = index;
    switch (index) {
      case 0:
        driveSparkMax = new CANSparkMax(DriveMotor.frontLeft.CAN_ID, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(TurnMotor.frontLeft.CAN_ID, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANCoder(8);
        absoluteEncoderOffset = -267.6269 + 180;
        break;
      case 1:
        driveSparkMax = new CANSparkMax(DriveMotor.frontRight.CAN_ID, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(TurnMotor.frontRight.CAN_ID, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANCoder(11);
        absoluteEncoderOffset = -269.7363 + 180;
        break;
      case 2:
        driveSparkMax = new CANSparkMax(DriveMotor.backLeft.CAN_ID, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(TurnMotor.backLeft.CAN_ID, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANCoder(5);
        absoluteEncoderOffset = -356.0449;
        break;
      case 3:
        driveSparkMax = new CANSparkMax(DriveMotor.backRight.CAN_ID, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(TurnMotor.backRight.CAN_ID, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANCoder(2);
        absoluteEncoderOffset = -177.6269;
        break;
      default:
        throw new RuntimeException("Invalid module index for ModuleIOSparkMax");
    }

    driveSparkMax.burnFlash();
    turnSparkMax.burnFlash();

    driveSparkMax.setCANTimeout(Constants.CANConfigTimeout);
    turnSparkMax.setCANTimeout(Constants.CANConfigTimeout);

    driveEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();

    for (int i = 0; i < DriveMotor.values().length; i++) {
      // SparkMaxPeriodicFrameConfig.configNotLeader(driveSparkMax);
      // SparkMaxPeriodicFrameConfig.configNotLeader(turnSparkMax);
      driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);

      turnSparkMax.setInverted(isTurnMotorInverted);

      driveSparkMax.setSmartCurrentLimit(40);
      turnSparkMax.setSmartCurrentLimit(30);
      driveSparkMax.enableVoltageCompensation(12.0);
      turnSparkMax.enableVoltageCompensation(12.0);

      driveEncoder.setPosition(0.0);
      driveEncoder.setMeasurementPeriod(10);
      driveEncoder.setAverageDepth(2);

      turnRelativeEncoder.setPosition(0.0);
      turnRelativeEncoder.setMeasurementPeriod(10);
      turnRelativeEncoder.setAverageDepth(2);
    }

    driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);

      driveSparkMax.burnFlash();
      turnSparkMax.burnFlash();
    
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad = 
        Units.rotationsToRadians(driveEncoder.getPosition()) / driveAfterEncoderReduction;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity())
            / driveAfterEncoderReduction;
    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] { driveSparkMax.getOutputCurrent() };
    inputs.driveTempCelcius = new double[] { driveSparkMax.getMotorTemperature() };

    inputs.turnAbsolutePositionRad = MathUtil.angleModulus(
        new Rotation2d(
            Math.toRadians(turnAbsoluteEncoder.getAbsolutePosition()
            - absoluteEncoderOffset))
            .getRadians());

            SmartDashboard.putNumber("absolute encoder" + nancy, Math.toRadians(turnAbsoluteEncoder.getAbsolutePosition()));
            SmartDashboard.putNumber("absolute encoder with offset" + nancy, Math.toRadians(turnAbsoluteEncoder.getAbsolutePosition() + absoluteEncoderOffset));
    inputs.turnPositionRad = 
        Units.rotationsToRadians(turnRelativeEncoder.getPosition())
            / turnAfterEncoderReduction;
    inputs.turnVelocityRadPerSec = 
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / turnAfterEncoderReduction;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] { turnSparkMax.getOutputCurrent() };
    inputs.turnTempCelcius = new double[] { turnSparkMax.getMotorTemperature() };
  }

  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  public double getAbsolutePosition(){
return turnAbsoluteEncoder.getAbsolutePosition();
  }
}
