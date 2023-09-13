package frc.robot.subsystems.pose;

import com.google.flatbuffers.Constants;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.Constants.ElevatorConstants;

public class Mechanisms2d {
    private MechanismLigament2d wrist2d;

    public final Mechanism2d swerveMech = new Mechanism2d(0.1,0.1); //TODO: Update Values
    public final MechanismRoot2d root = swerveMech.getRoot("MechanismRoot", 1, 1); 

    public final MechanismLigament2d elevator2d = root.append(new MechanismLigament2d("elevator", ElevatorConstants.elevatorStartingConfigLengthInches, ElevatorConstants.elevatorAngleDegrees));
    }
