// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.EndEffectorConstants;

/** Add your docs here. */
public class EndEffectorIOSim implements EndEffectorIO{
    private final EndEffectorSim endEffectorMotor = new FlywheelSim(DCMotor.getNEO(1), EndEffectorConstants.gearRatio,0.025);
    
}
