// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopAutos;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.wrist.Wrist;

public class MoveWristPosition extends CommandBase {
  /** Creates a new MoveWristPosition. */
  private double m_endPoint; 
  private double m_startPoint;  
  private double m_speed; 
  private Wrist m_wristSubsystem;

  public MoveWristPosition(double endPoint, double speed, Wrist wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_endPoint = endPoint; 
    m_speed = speed; 
    m_wristSubsystem = wrist; 
    addRequirements(m_wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startPoint = m_wristSubsystem.getWristPositionDegrees();
    m_wristSubsystem.setWristPercentSpeed(0.0);
    Logger.getInstance().recordOutput("get endpoint wrist", m_endPoint);
    Logger.getInstance().recordOutput("get startpoint wrist", m_startPoint);
    System.out.println("init move wrist");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_wristSubsystem.getWristPositionDegrees() > m_endPoint) {
      m_wristSubsystem.setWristPercentSpeed(-m_speed);
    } else if( m_wristSubsystem.getWristPositionDegrees() < m_endPoint) {
      m_wristSubsystem.setWristPercentSpeed(m_speed);
    } else {
      m_wristSubsystem.setWristPercentSpeed(0.0);
    }
    System.out.println("exicute move wrist");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wristSubsystem.setWristPercentSpeed(0.0);
    System.out.println("END");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.getInstance().recordOutput("get endpoint wrist", m_endPoint);
    Logger.getInstance().recordOutput("get startpoint wrist", m_startPoint);

    return (m_startPoint > m_endPoint) ? m_wristSubsystem.getWristPositionDegrees() <= m_endPoint :
    m_wristSubsystem.getWristPositionDegrees() >= m_endPoint;
  }
}
