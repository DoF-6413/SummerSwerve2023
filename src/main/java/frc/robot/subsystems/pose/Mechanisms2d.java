package frc.robot.subsystems.pose;

import org.littletonrobotics.junction.Logger;
import com.google.flatbuffers.Constants;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Mechanisms2d extends SubsystemBase{
    private MechanismLigament2d wrist2d;

    public final Mechanism2d swerveMech; 
    public final MechanismRoot2d root;

    public final MechanismLigament2d elevator2d = root.append(new MechanismLigament2d("elevator", 
        ElevatorConstants.elevatorStartingConfigLengthInches, ElevatorConstants.elevatorAngleDegrees));
    
        public Mechanisms2d() {
          root = swerveMech.getRoot("MechanismRoot", 1, 1); 
          swerveMech = new Mechanism2d(0.1,0.1); //TODO: Update Values
        wrist2d = elevator2d.append(new MechanismLigament2d("elevator", ElevatorConstants.elevatorStartingConfigLengthInches));
        SmartDashboard.putData("Mech2d", Mech2d);
        Logger.getInstance().recordOutput("FPGA TIme", Timer.getFPGATimestamp());

    }

    }
