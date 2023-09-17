// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
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
        endEffectorIO = io;
        endEffectorPIDController =
        new ProfiledPIDController(
            EndEffectorConstants.endEffectorkP,
            EndEffectorConstants.endEffectorkI, 
            EndEffectorConstants.endEffectorkD, 
            new TrapezoidProfile.Constraints(
                EndEffectorConstants.maxVelocity, 
                EndEffectorConstants.maxAcceleration)
        );

        endEffectorPIDController.setTolerance(EndEffectorConstants.positionTolerance, EndEffectorConstants.velocityTolerance);
    }
    
    public void periodic() {
        endEffectorIO.updateInputs(endEffectorInputs);
        Logger.getInstance().processInputs("EndEffector", endEffectorInputs);
        Logger.getInstance().recordOutput("Elevator/PositionMeters", getEndEffectorPositionMeters());
        Logger.getInstance().recordOutput("Elevator/Voltage", getEndEffectorVoltage());
        Logger.getInstance().recordOutput("Elevator/PositionGoal", endEffectorPIDController.getGoal().position);
    }

    public double getEndEffectorPositionMeters(){
        return endEffectorInputs.endEffectorPositionRad;
    }

    public double getEndEffectorVelocityMetersPerSec() {
        return endEffectorInputs.endEffectorPositionRad * Math.PI * Units.inchesToMeters(EndEffectorConstants.velocityTolerance);
    }

    public double getEndEffectorVoltage() {
        return endEffectorInputs.endEffectorAppliedVolts;
    }

    public void setVoltage(double volts){
        endEffectorIO.setVoltage(volts);
    }

    public void setPercentSpeed(double percent){
        endEffectorIO.setVoltage(percent * 12);
        System.out.println("HI");
    }
}











