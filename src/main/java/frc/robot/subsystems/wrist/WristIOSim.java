// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

import frc.robot.Constants.WristNeo550Constants;

public class WristIOSim implements WristIO {
    private FlywheelSim WristMotor = new FlywheelSim(DCMotor.getNeo550(1),WristNeo550Constants.gearRatio,1);

    public WristIOSim(){
         System.out.println("[Init] Creating WristIOSim");
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        WristMotor.update(Constants.loopPeriodSecs);
        inputs.turnPositionRad += WristMotor.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs;
        inputs.turnVelocityRadPerSec = WristMotor.getAngularVelocityRadPerSec();
        inputs.turnAppliedVolts = 0.0;
        inputs.turnCurrentAmps = Math.abs(WristMotor.getCurrentDrawAmps()); 
        inputs.WristTempCelcius = 0.0;
    }
        
    

    public void setWristSpeed(Double speed) {
        WristMotor.setInputVoltage(speed * WristNeo550Constants.WristAppliedVolts);
    }
}