// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.QuickAuto;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.subsystems.gyro.GyroIO;
import frc.robot.subsystems.gyro.GyroIONavX;
import frc.robot.subsystems.gyro.GyroIOSim;
import frc.robot.subsystems.pose.Pose;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOArduCam;
import frc.robot.subsystems.vision.VisionIOSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.moduleIO;
import frc.robot.commands.AutoDriver;
import frc.robot.Trajectories;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  // private final Flywheel flywheel;
  private final Gyro gyro;
  private final Vision vision;
  private final Pose pose;
  
  // Controller
  private final CommandXboxController controller = new CommandXboxController(OperatorConstants.DriveController);
  
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  
  public SendableChooser<Command> m_Chooser = new SendableChooser<>();
  public RobotContainer() {
    switch (Constants.getMode()) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
      System.out.println("Robot Current Mode; REAL");
      gyro = new Gyro(new GyroIONavX());
      drive = new Drive(new ModuleIOSparkMax(0), new ModuleIOSparkMax(1), new ModuleIOSparkMax(2), new ModuleIOSparkMax(3), gyro);
      vision = new Vision(new VisionIOArduCam());
      pose = new Pose(drive, gyro, vision, drive.swerveKinematics);
      break;
      
      // Sim robot, instantiate physics sim IO implementations
      case SIM:
      System.out.println("Robot Current Mode; SIM");
      // drive = new Drive(new DriveIOSim());
      gyro = new Gyro(new GyroIOSim());
      drive = new Drive(new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), gyro);
        vision = new Vision(new VisionIOSim());
        pose = new Pose(drive, gyro, vision, drive.swerveKinematics);

        // flywheel = new Flywheel(new FlywheelIOSim());
        break;
        
        // Replayed robot, disable IO implementations
        default:
        System.out.println("Robot Current Mode; default");
        // flywheel = new Flywheel(new FlywheelIO() {});
        gyro = new Gyro(new GyroIO(){});
        drive = new Drive(new moduleIO() {}, new moduleIO() {}, new moduleIO() {}, new moduleIO() {}, gyro);
        vision = new Vision(new VisionIO() {});
        pose = new Pose(drive, gyro, vision, drive.swerveKinematics);
        break;
        
      }
      
      m_Chooser.addOption("FullAuto", new AutoDriver(drive, gyro, pose, Trajectories.test, true));
      // Set up auto routines
      autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
      // autoChooser.addOption("FullAuto", new AutoDriver(drive, gyro, pose, Trajectories.test, true));
      // SmartDashboard.putData(m_Chooser);
      Shuffleboard.getTab("Auto").add(autoChooser.getSendableChooser());
      
      
      
     
        
        // Configure the button bindings
        configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
      new DefaultDriveCommand(drive, gyro,()->-controller.getLeftY(), ()->-controller.getLeftX(), ()->-controller.getRightX()));

      controller.a().onTrue(new InstantCommand(()-> gyro.updateHeading(), gyro));
    }
    
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
     return autoChooser.get();
    //  return new QuickAuto(drive, gyro, 4);

    
  }
}
