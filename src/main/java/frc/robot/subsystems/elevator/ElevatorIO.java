// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {

    @AutoLog
    public static class ElevatorIOInputs{
        public double elevatorPositionRad = 0.0;
        public double elevatorVelocityRadPerSec = 0.0;
        public boolean limitSwitchPress 
        public double[] elevatorCurrentAmps = new double[] {};
        public double[] elevatorTempCelcius = new double[] {};

    }
}

public default void updateInputs(ElevatorIOInputs inputs) {}