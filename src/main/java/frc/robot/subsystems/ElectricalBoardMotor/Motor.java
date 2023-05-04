// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ElectricalBoardMotor;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectricalBoardMotorConstants;

/** Add your docs here. */
public class Motor extends SubsystemBase {
    private final MotorIO io;
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    public Motor(MotorIO io){
        this.io = io;
    }

    @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Motor", inputs);
  }

  public void spinPercent(double percent){
    io.setVoltage(percent * 12.0);
  }

  public void stop(){
    io.setVoltage(0);
  }

  public double getPositionMeters(){
    return inputs.motorPositionRad * ElectricalBoardMotorConstants.kMotorDiameter;
  }

  public double getPositionRads(){
    return inputs.motorPositionRad;
  }

  public double getVelocityMetersPerSec(){
    return inputs.motorVelocityRadPerSec * ElectricalBoardMotorConstants.kMotorDiameter;
  }

  public double getVelocityRadsPerSec(){
    return inputs.motorVelocityRadPerSec;
  }
}
