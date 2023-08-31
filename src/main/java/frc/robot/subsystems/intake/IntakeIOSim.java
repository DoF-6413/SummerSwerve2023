// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

/** Add your docs here. */
public class IntakeIOSim implements IntakeIO {
    private FlywheelSim intakeSim = new FlywheelSim(DCMotor.getNEO(2), IntakeConstants.gearRatio, 0); //TODO: what is jKgMetersSquared?????
    
    private double intakeAppliedVolts = 0.0;
    public IntakeIOSim() {

    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakePositionRad += intakeSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs;

    }
}
