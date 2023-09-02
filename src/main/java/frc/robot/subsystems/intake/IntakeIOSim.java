// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeMotor;

/** Add your docs here. */
public class IntakeIOSim implements IntakeIO {
    private FlywheelSim intakeMotorSim = new FlywheelSim(DCMotor.getNEO(2), IntakeConstants.gearRatio, 0); //TODO: what is jKgMetersSquared?????
    
    private double intakeAppliedVolts = 0.0; //TODO find volts ??
    public IntakeIOSim() {
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakePositionRad += intakeMotorSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs;
        inputs.intakeVelocityRadPerSec = intakeMotorSim.getAngularVelocityRadPerSec();
        inputs.intakeAppliedVolts = 0.0;
        inputs.intakeCurrentAmps = new double[] {Math.abs(intakeMotorSim.getCurrentDrawAmps())};
        inputs.intakeTempCelcius = new double[] {};        
    }

    public void setPercentSpeed(double percent) {
        intakeMotorSim.setInputVoltage(percent * intakeAppliedVolts);
    }
}
