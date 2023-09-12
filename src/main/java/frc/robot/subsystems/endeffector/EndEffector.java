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
    public final Mechanism2d endEffector2d = new Mechanism2d(0.2,0.2);
    public final MechanismRoot2d endEffectorMechanismRoot2d = endEffector2d.getRoot("endEffectorRoot",1,1);
    
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
      Logger.getInstance().recordOutput("endEffector2d", endEffector2d); 
    }
    
    public void periodic() {
        endEffectorIO.updateInputs(endEffectorInputs);
        Logger.getInstance().processInputs("EndEffector", endEffectorInputs);
        Logger.getInstance().recordOutput("Elevator/PositionMeters", getEndEffectorPositionMeters());
        Logger.getInstance().recordOutput("Elevator/Voltage", getEndEffectorVoltage());
        Logger.getInstance().recordOutput("Elevator/PositionGoal", endEffectorPIDController.getGoal().position);
        endEffector2d.
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
        endEffectorIO.setPercentSpeed(percent);
    }
}











