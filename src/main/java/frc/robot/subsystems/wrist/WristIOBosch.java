// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PWM;
import frc.robot.Constants.WristboschConstants;

/** Add your docs here. */
public class WristIOBosch implements WristIO {

    private final CANSparkMax wristBosch;

    public WristIOBosch() {
        wristBosch = new CANSparkMax(WristboschConstants.boschCanID, MotorType.kBrushed);
    }

    public void setWristSpeed(int speed) {
        // the speed should be a number between -1.0 to 1.0
        wristBosch.setVoltage(speed);
    }

    public void updateInputs(WristIOInputs inputs){
        //  inputs.turnAppliedVolts = wristAI.getVoltage();
        //  inputs.turnPositionRad = Units.rotationsToRadians(wristAI.getAccumulatorCount()) / WristboschConstants.gearRatio;
    }

}
