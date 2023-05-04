// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ElectricalBoardMotor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ElectricalBoardMotorConstants;

/** Add your docs here. */
public class MotorIOSparkMax implements MotorIO {
    private final CANSparkMax electricalBoardMotor;
    private final RelativeEncoder electricalBoardEncoder;  

    public MotorIOSparkMax(){
        electricalBoardMotor = new CANSparkMax(ElectricalBoardMotorConstants.kMotorCANID, MotorType.kBrushless);
        electricalBoardEncoder =electricalBoardMotor.getEncoder();

        electricalBoardMotor.restoreFactoryDefaults();
        electricalBoardMotor.setInverted(false);
        //Keeps the Motor Output below set Voltage
        electricalBoardMotor.enableVoltageCompensation(12.0);
        //Amps
        electricalBoardMotor.setSmartCurrentLimit(30);

        electricalBoardMotor.burnFlash();
        }

        @Override
        public void updateInputs(MotorIOInputs inputs) {
            inputs.motorPositionRad = Units.rotationsToRadians(electricalBoardEncoder.getPosition()/ ElectricalBoardMotorConstants.kGearRatio);
            inputs.motorVelocityRadPerSec = Units.rotationsToRadians(electricalBoardEncoder.getVelocity()/ ElectricalBoardMotorConstants.kGearRatio);
            inputs.motorCurrentAmps = electricalBoardMotor.getOutputCurrent();
            inputs.motorAppliedVolts = electricalBoardMotor.getAppliedOutput();               
        }
      
        @Override
        public void setVoltage(double motorVolts) {
          electricalBoardMotor.setVoltage(motorVolts);
        }
}
