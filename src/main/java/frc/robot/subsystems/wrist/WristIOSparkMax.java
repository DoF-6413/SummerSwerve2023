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
public class WristIOSparkMax implements WristIO {

    private final CANSparkMax wristCanSparkMax;
    // private final AnalogInput wristAI;

    public WristIOSparkMax() {
        wristCanSparkMax = new CANSparkMax(16,MotorType.kBrushed);
        // wristAI = new AnalogInput(WristboschConstants.wristAnalogInput);
    }

    public void setWristSpeed(Double speed) {
        // the speed should be a number between -1.0 to 1.0
        wristCanSparkMax.set(speed);

    }

    public void updateInputs(WristIOInputs inputs){
         inputs.turnCurrentAmps = wristCanSparkMax.getOutputCurrent();
         wristCanSparkMax.setSmartCurrentLimit(5);
       //  inputs.turnPositionRad = Units.rotationsToRadians(wristCanSparkMax.) / WristboschConstants.gearRatio; TODO:update
    }

}
