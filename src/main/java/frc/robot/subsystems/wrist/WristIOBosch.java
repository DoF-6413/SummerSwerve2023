// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj.AnalogInput;

/** Add your docs here. */
public class WristIOBosch implements WristIO {
    private final AnalogInput wristMotor; 
    public WristIOBosch(){
       wristMotor = new AnalogInput(0);
    }
    @Override
    public void setVoltageSpeed(double volts) {
      wristMotor.set    }
}

