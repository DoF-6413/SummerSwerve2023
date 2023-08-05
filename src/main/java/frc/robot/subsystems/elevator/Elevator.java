// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

/** Add your docs here. */
public class Elevator extends SubsystemBase{
    public static ElevatorIO elevatorIO;
    public static ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

    public Elevator(ElevatorIO io) {
        System.out.println("[Init] Creating Elevator ");
        elevatorIO = io;
        
      }

      public void periodic() {
        elevatorIO.updateInputs(elevatorInputs);
        Logger.getInstance().processInputs("Elevator", elevatorInputs);
    }

    public boolean isLimitSwitchPressed(){
        return elevatorInputs.limitSwitchPressed;
    }
    /** extension of elevator (hypotenuse)
     */
    public double getElevatorPositionMeters(){
        return elevatorInputs.elevatorPositionRad * Math.PI * Units.inchesToMeters(ElevatorConstants.shaftDiameterInches);
    } 
    /** height of end of elevator off the ground
     */
    public double getElevatorHeightMeters(){
        return Math.sin(ElevatorConstants.elevatorAngleDegrees) * (getElevatorPositionMeters()) + Units.inchesToMeters(ElevatorConstants.elevatorStartingConfigHeightInches);
    }
    
    public double getElevatorVelocityMetersPerSec(){
        return elevatorInputs.elevatorVelocityRadPerSec;
    }

    /** extension in feet
     */
    public double getElevatorPositionFeet(){
        return elevatorInputs.elevatorPositionRad * ElevatorConstants.shaftDiameterInches * Math.PI / 12;
    }    
    /** height off of the ground
     */
    public double getElevatorHeightFeet(){
        return Units.metersToFeet(getElevatorHeightMeters());
    }    

    public void setElevatorPercentSpeed(double percent){
        elevatorIO.setPercentSpeed(percent);
    }
}
