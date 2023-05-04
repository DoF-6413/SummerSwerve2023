// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveWithFlywheelAuto;
import frc.robot.commands.SpinAuto;
import frc.robot.subsystems.ElectricalBoardMotor.Motor;
import frc.robot.subsystems.ElectricalBoardMotor.MotorIO;
import frc.robot.subsystems.ElectricalBoardMotor.MotorIOSparkMax;
import frc.robot.subsystems.driveExample.*;
import frc.robot.subsystems.driveExample.DriveIO;
import frc.robot.subsystems.driveExample.DriveIOSim;
import frc.robot.subsystems.driveExample.DriveIOSparkMax;
import frc.robot.subsystems.flywheelExample.Flywheel;
import frc.robot.subsystems.flywheelExample.FlywheelIO;
import frc.robot.subsystems.flywheelExample.FlywheelIOSim;
import frc.robot.subsystems.flywheelExample.FlywheelIOSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  // private final Drive drive;
  // private final Flywheel flywheel;
  private final Motor motor;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");
  private final LoggedDashboardNumber flywheelSpeedInput = new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
      motor = new Motor(new MotorIOSparkMax());
        // drive = new Drive(new DriveIOSparkMax());
        // flywheel = new Flywheel(new FlywheelIOSparkMax());
        // drive = new Drive(new DriveIOFalcon500());
        // flywheel = new Flywheel(new FlywheelIOFalcon500());
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        // drive = new Drive(new DriveIOSim());
        // flywheel = new Flywheel(new FlywheelIOSim());
      motor = new Motor(new MotorIO() {});
        break;

      // Replayed robot, disable IO implementations
      default:
      motor = new Motor(new MotorIOSparkMax());
        // drive = new Drive(new DriveIO() {
        // });
        // flywheel = new Flywheel(new FlywheelIO() {
        // });
        break;
    }
    // Set up auto routines
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    // autoChooser.addOption("Spin", new SpinAuto(drive));
    // autoChooser.addOption("Drive With Flywheel", new DriveWithFlywheelAuto(drive, flywheel));

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
    // drive.setDefaultCommand(
    //     new RunCommand(() -> drive.driveArcade(-controller.getLeftY(), controller.getLeftX()), drive));
    // controller.a()
    //     .whileTrue(new StartEndCommand(() -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel));

    motor.setDefaultCommand(new RunCommand(()-> motor.spinPercent(-controller.getLeftY()), motor));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
