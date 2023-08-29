// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Add your docs here. */
public interface IntakeIO{

    @AutoLog
    public static class IntakeIOInputs implements LoggableInputs {
        public double intakePositionRad = 0.0;
        public double intakeVelocityRadPerSec = 0.0;
        public double intakeAppliedVolts = 0.0;
        public double[] elevatorCurrentAmps = new double[] {};
        public double[] elevatorTempCelcius = new double[] {};
        @Override
        public void toLog(LogTable table) {
            // TODO Auto-generated method stub
            
        }
        @Override
        public void fromLog(LogTable table) {
            // TODO Auto-generated method stub
            
        }
    }

public default void updateInputs(IntakeIOInputs inputs) {}

public default void setPercentSpeed(double voltage) {}
    
}

