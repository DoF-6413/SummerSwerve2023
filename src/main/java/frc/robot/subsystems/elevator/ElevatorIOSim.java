// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorMotor;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO {
    private final ElevatorSim elevatorMotor = new ElevatorSim(
        DCMotor.getFalcon500(2),
        ElevatorConstants.gearRatio,
        Units.lbsToKilograms(ElevatorConstants.carriageMassPounds),
        Units.inchesToMeters(ElevatorConstants.shaftDiameterInches / 2),
        Units.inchesToMeters(ElevatorConstants.elevatorStartingConfigHeightInches),
        Units.inchesToMeters(ElevatorConstants.elevatorFullExtensionHeightInches),
        ElevatorConstants.simulateGravity
    );
   private double elevatorAppliedVolts = 0.0;
    public ElevatorIOSim() {

    }
    
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.elevatorPositionRad =
            Units.rotationsToRadians(elevatorMotor.getVelocityMetersPerSecond() / ElevatorConstants.gearRatio);
        inputs.elevatorVelocityRadPerSec = 
            Units.rotationsPerMinuteToRadiansPerSecond(elevatorAppliedVolts) / ElevatorConstants.gearRatio;
        inputs.limitSwitchPressed = false;
        inputs.elevatorAppliedVolts = 0.0;
        inputs.elevatorCurrentAmps = new double[] {elevatorMotor.getCurrentDrawAmps()};
        inputs.elevatorTempCelcius = new double[] {};
    }

    @Override
    public void setVoltageSpeed(double volts) {
        elevatorMotor.setInputVoltage(volts);
      }
}
