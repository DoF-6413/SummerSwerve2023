// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;

/** Add your docs here. */
public class EndEffectorIOSim implements EndEffectorIO{
    private FlywheelSim endEffectorMotor = new FlywheelSim(DCMotor.getNEO(1), EndEffectorConstants.gearRatio,0.0);

    public EndEffectorIOSim() {
        System.out.println("[Init] Creating EndEffectorIOSim");
    }
    
    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        endEffectorMotor.update(Constants.loopPeriodSecs);
        
        inputs.endEffectorPositionRad += endEffectorMotor.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs;
        inputs.endEffectorVelocityRadPerSec = endEffectorMotor.getAngularVelocityRadPerSec();
        inputs.endEffectorAppliedVolts = 0.0;
        inputs.endEffectorCurrentAmps = new double[] {Math.abs(endEffectorMotor.getCurrentDrawAmps())};
        inputs.endEffectorTempCelcius = new double[] {};
    }

    public void setPercentSpeed(double percent) {
        endEffectorMotor.setInputVoltage(percent * EndEffectorConstants.endeffectorAppliedVolts);
    }
}
