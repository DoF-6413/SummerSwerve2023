// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristNeoConstants;

/** Add your docs here. */
public class Wrist extends SubsystemBase {

    public static WristIO wristIO;
    public static WristIOInputsAutoLogged WristInputs = new WristIOInputsAutoLogged();
    public ProfiledPIDController wristPIDController;


    public Wrist(WristIO io){

        System.out.println("[Init] creating wrist");
        wristIO = io;
        wristPIDController =
        new ProfiledPIDController(
            WristNeoConstants.WristkP,
            WristNeoConstants.WristkI, 
            WristNeoConstants.WristkD, 
            new TrapezoidProfile.Constraints(
               WristNeoConstants.maxVelocity, 
               WristNeoConstants.maxAcceleration)
        );
    }

 public void periodic(){
    wristIO.updateInputs(WristInputs);
    Logger.getInstance().processInputs("Wrist", WristInputs);
 }
 public double getWristPositionMeters(){
    return WristInputs.turnPositionRad * 2*Math.PI; //* Units.inchesToMeters(WristConstants.shaftDiameterInches);
 } 
 public void setWristSpeed(Double voltage){
        wristIO.setWristSpeed(voltage);


}

public double getWristPositionDegrees(){
   return Units.radiansToDegrees(WristInputs.turnPositionRad);
}
public void setWristPercentSpeed(Double percent){
   wristIO.setWristSpeed(percent * 12);


}
}
