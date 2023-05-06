package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class NancysSuperFancyAutonomousCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    public NancysSuperFancyAutonomousCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }
@Override 
public void initialize() {
  m_drivetrainSubsystem.resetGyroscopeXY();
}
    
    @Override
    public void execute() {

        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.drive(
            //non-gyro drive (working)
        //     new ChassisSpeeds(
        // m_translationXSupplier.getAsDouble(), 
        //  m_translationYSupplier.getAsDouble(), 
        //  m_rotationSupplier.getAsDouble()));

         //gyro drive (WIP)
                ChassisSpeeds.fromFieldRelativeSpeeds(
                       m_translationXSupplier.getAsDouble(), 
                         m_translationYSupplier.getAsDouble(), 
                        m_rotationSupplier.getAsDouble(),
                        m_drivetrainSubsystem.getGyroscopeRotation()
                        ));
    }

    @Override
    public void end(boolean interrupted) {
 m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, 0.0, m_drivetrainSubsystem.getGyroscopeRotation() ));
    }   

      // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drivetrainSubsystem.readNavxY() < -0.62;
  }
}

