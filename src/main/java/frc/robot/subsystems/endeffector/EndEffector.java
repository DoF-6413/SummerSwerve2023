// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.endeffector.EndEffectorIO.EndEffectorIOInputs;

/** Add your docs here. */
public class EndEffector extends SubsystemBase {
    public static EndEffectorIO endEffectorIO;
    public static EndEffectorIOInputsAutoLogged endEffectorInputs = new EndEffectorIOInputsAutoLogged();
    public ProfiledPIDController endEffectorPIDController;
       

public EndEffector(EndEffectorIO io) {
    System.out.println("[Init] Creating EndEffector");
    //     endEffectorIO = io;
    //     endEffectorPIDController =
    //     new ProfiledPIDController(
    //         EndEffectorCons endEffectorkP,
    //         0, 
    //         0, 
    //         null)
    }
    
    public void periodic() {
        endEffectorIO.updateInputs(endEffectorInputs);
        Logger.getInstance().processInputs("EndEffector", endEffectorInputs);
    }

    public double getPositionMeters(){
        return endEffectorInputs.endeffectorPositionRad;
    }

    public void setVoltage(double volts){
        endEffectorIO.setVoltage(volts);
    }

    public void setPercentSpeed(double percent){
        endEffectorIO.setPercentSpeed(percent);
    }
}











