// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PWM;

/** Add your docs here. */
public class WristIOBosch implements WristIO {
   
    private final PWM wristPwm;
   
    public WristIOBosch(){
       wristPwm = new PWM(2);
    }


    public void setWristSpeed(double speed) {
//the speed should be a number between -1.0 to 1.0
wristPwm.setSpeed(speed);

       }
}

