package frc.robot.subsystems.pose;

import org.littletonrobotics.junction.Logger;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Mechanisms2d extends SubsystemBase {

  private final Mechanism2d swerveMech;
  private final MechanismRoot2d root;
  private final MechanismLigament2d elevatorMech;
  private final MechanismLigament2d wristMech;
  private final Elevator elevatorSub;
  private final Wrist wristSub;

  public Mechanisms2d(Elevator elevatorSub, Wrist wristSub) {
    this.elevatorSub = elevatorSub;
    this.wristSub = wristSub;
    swerveMech = new Mechanism2d(3,3); // TODO: Update Values
    root = swerveMech.getRoot("MechanismRoot", 1, 1);



    elevatorMech = root.append(
        new MechanismLigament2d(
            "ElevatorRoot",
            Units.inchesToMeters(ElevatorConstants.elevatorStartingConfigLengthInches),
            ElevatorConstants.elevatorAngleDegrees,10,new Color8Bit(0,255,0)));



    wristMech = elevatorMech.append(
        new MechanismLigament2d(
            "WristRoot",
            0.4,
            0));
    SmartDashboard.putData("Swerve Mech", swerveMech);
    
    
  }
  
  @Override
  public void periodic() {
    // set the 
    elevatorMech.

    
    setLength(


    Units.inchesToMeters(ElevatorConstants.elevatorStartingConfigLengthInches) //the minimun length of the elevator 
    + 
    elevatorSub.getElevatorPositionMeters()); //position of the elevator
// 
    
    wristMech.setAngle(wristSub.getWristPositionMeters());
    Logger.getInstance().recordOutput("SwerveMech", swerveMech);
  }
}
