// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Add your docs here. */
public interface ElevatorIO {

    @AutoLog
    public static class ElevatorIOInputs implements LoggableInputs {
        public double elevatorPositionRad = 0.0;
        public double elevatorVelocityRadPerSec = 0.0;
        public boolean limitSwitchPressed = false;
        public double elevatorAppliedVolts = 0.0;
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


public default void updateInputs(ElevatorIOInputs inputs) {}

public default void setVoltageSpeed(double volts) {}
}