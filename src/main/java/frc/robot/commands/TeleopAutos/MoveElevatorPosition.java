// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopAutos;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.Elevator;

public class MoveElevatorPosition extends CommandBase {
  /** Creates a new MoveElevatorPosition. */
  private double m_endPoint;
  private double m_startPoint;
  private double m_speed;
  private Elevator m_elevatorSubsystem;

  public MoveElevatorPosition(double endPoint, double speed, Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_endPoint = endPoint; 
    m_speed = speed;
    m_elevatorSubsystem = elevator;
    addRequirements(m_elevatorSubsystem);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startPoint = m_elevatorSubsystem.getElevatorPositionFeet();
    m_elevatorSubsystem.setElevatorPercentSpeed(0);
    Logger.getInstance().recordOutput("get endpoint", m_endPoint);
    Logger.getInstance().recordOutput("get startpoint", m_startPoint);
    System.out.println("init move elevator");
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_elevatorSubsystem.getElevatorPositionFeet() > m_endPoint) {
      m_elevatorSubsystem.setElevatorPercentSpeed(-m_speed);
    } else if (m_elevatorSubsystem.getElevatorPositionFeet() < m_endPoint) {
      m_elevatorSubsystem.setElevatorPercentSpeed(m_speed);
    } else {
      m_elevatorSubsystem.setElevatorPercentSpeed(0);
    }

    System.out.println("exicute move elevator");
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.setElevatorPercentSpeed(0);
    System.out.println("END");
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.getInstance().recordOutput("get endpoint", m_endPoint);
    Logger.getInstance().recordOutput("get startpoint", m_startPoint);

    return (m_startPoint > m_endPoint) ? m_elevatorSubsystem.getElevatorHeightFeet() <= m_endPoint : 
    m_elevatorSubsystem.getElevatorHeightFeet() >= m_endPoint ;
  }
}
