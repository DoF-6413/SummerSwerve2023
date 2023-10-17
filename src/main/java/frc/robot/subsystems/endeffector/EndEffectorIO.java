// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface EndEffectorIO{
/** Add your docs here. */
@AutoLog
public static class EndEffectorIOInputs{
    public double endEffectorPositionRad = 0.0;
    public double endEffectorVelocityRadPerSec = 0.0;
    public double endEffectorAppliedVolts = 0.0;
    public double[] endEffectorCurrentAmps = new double[] {};
    public double[] endEffectorTempCelcius = new double[] {};

}

public default void updateInputs(EndEffectorIOInputs inputs) {}

public default void setVoltage(double voltage){}

public default void setPercentSpeed(double percent){}

}
