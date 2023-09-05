package frc.robot.subsystems.drive;

import java.util.Arrays;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.subsystems.gyro.GyroIO;
import frc.robot.subsystems.gyro.GyroIO.GyroIOInputs;
import frc.robot.subsystems.vision.VisionIOSim;

public class Drive extends SubsystemBase {

  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final VisionIOSim visionIOSim = new VisionIOSim();
  private static final Module[] modules = new Module[4];
  private final Gyro gyro;

  private double maxAngularSpeed;

  //TODO: Ensure this is where we want to store and access SwerveDriveKinematics
  public SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(getModuleTranslations());

  private ChassisSpeeds setpoint = new ChassisSpeeds();
  private SwerveModuleState[] lastSetpointStates = new SwerveModuleState[] {
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState()
  };

  private Timer lastMovementTimer = new Timer();

  /** Creates a new Drive. */
  public Drive(moduleIO flModuleIO, moduleIO frModuleIO, moduleIO blModuleIO, moduleIO brModuleIO, Gyro gyro) {

    System.out.println("[Init] Creating Drive");
    this.gyro = gyro;

    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);
    lastMovementTimer.start();
    for (var module : modules) {
      module.setBrakeMode(true);
      
    }
    
  }

  @Override
  public void periodic() {

    for(int i = 0; i < 4; i++){
      modules[i].periodic();
    }
    
    // Run modules
    if (DriverStation.isDisabled()) {
      // Stop moving while disabled
      for (var module : modules) {
        module.stop();
      }
    }

    swerveKinematics = new SwerveDriveKinematics(getModuleTranslations());
    maxAngularSpeed = DrivetrainConstants.maxLinearSpeed
        / Arrays.stream(getModuleTranslations())
            .map(translation -> translation.getNorm())
            .max(Double::compare)
            .get();


    //Clears the setpoint logs 
    Logger.getInstance().recordOutput("SwerveStates/Setpoints", new double[] {});
    Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", new double[] {});
   
        
              
      SwerveModuleState[] setpointStates = swerveKinematics.toSwerveModuleStates(setpoint);
      SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DrivetrainConstants.maxLinearSpeed);

      // // Send setpoints to modules
      SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
      for (int i = 0; i < 4; i++) {
        optimizedStates[i] = modules[i].runSetpoint(setpointStates[i]);
      }

      // // Log setpoint states
      Logger.getInstance().recordOutput("SwerveStates/Setpoints", setpointStates);

      Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", optimizedStates);


          // Log measured states
    SwerveModuleState[] measuredStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      measuredStates[i] = modules[i].getState();
    }

    Logger.getInstance().recordOutput("SwerveStates/Measured", measuredStates);

 
  }


  public void runVelocity(ChassisSpeeds speeds) {
    DrivetrainConstants.ischaracterizing = false;
    setpoint = speeds;
  }

  public void setRaw(double x, double y, double rot) {
    runVelocity(
      ChassisSpeeds.fromFieldRelativeSpeeds(
      x, 
      y,
      rot, 
        gyro.getYaw())
    );
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  // stops drive by turning the modules to an X arrangement to resist movement.
  public void stopWithX() {
    stop();
    for (int i = 0; i < 4; i++) {
      lastSetpointStates[i] = new SwerveModuleState(lastSetpointStates[i].speedMetersPerSecond,
          getModuleTranslations()[i].getAngle());
    }
  }

  // returns max linear speeds in terms of meters per sec
  public double getMaxLinearSpeedMetersPerSec() {
    return DrivetrainConstants.maxLinearSpeed;
  }

  // returns max angular speed in radians per second
  public double getMaxAngularSpeedRadPerSec() {
    return maxAngularSpeed;
  }


  /** Returns an array of module translations. */
  public Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
        new Translation2d(DrivetrainConstants.trackWidthX / 2.0, DrivetrainConstants.trackWidthY / 2.0),
        new Translation2d(DrivetrainConstants.trackWidthX / 2.0, -DrivetrainConstants.trackWidthY / 2.0),
        new Translation2d(-DrivetrainConstants.trackWidthX / 2.0, DrivetrainConstants.trackWidthY / 2.0),
        new Translation2d(-DrivetrainConstants.trackWidthX / 2.0, -DrivetrainConstants.trackWidthY / 2.0)
    };
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
      modules[0].getPosition(),
      modules[1].getPosition(),
      modules[2].getPosition(),
      modules[3].getPosition()
    };
  }

  public void useModuleStates(SwerveModuleState[] states){
    //Declare a var where you coerce states to chasis speed type
    //Pass into run velocity
    swerveKinematics.toChassisSpeeds(states);
    }
}
