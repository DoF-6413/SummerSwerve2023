package frc.robot.subsystems.drive;

import java.util.Arrays;

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
  private SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(getModuleTranslations());

  private ChassisSpeeds setpoint = new ChassisSpeeds();
  private SwerveModuleState[] lastSetpointStates = new SwerveModuleState[] {
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState()
  };

  private Timer lastMovementTimer = new Timer();

  // private PoseEstimator poseEstimator = new
  // PoseEstimator(VecBuilder.fill(0.003, 0.003, 0.002));
  private double[] lastModulePositionsMeters = new double[] { 0.0, 0.0, 0.0, 0.0 };
  private Rotation2d lastGyroYaw = new Rotation2d();
  private Twist2d fieldVelocity = new Twist2d();

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
      module.setBrakeMode(false);
      
    }
    
  }

  @Override
  public void periodic() {

    for (var module : modules) {
      module.periodic();
    }

    Logger.getInstance().processInputs("Drive", inputs);
    swerveKinematics = new SwerveDriveKinematics(getModuleTranslations());
    maxAngularSpeed = DrivetrainConstants.maxLinearSpeed
        / Arrays.stream(getModuleTranslations())
            .map(translation -> translation.getNorm())
            .max(Double::compare)
            .get();

  // Run modules
  if (DriverStation.isDisabled()) {
    // Stop moving while disabled
    for (var module : modules) {
      module.stop();
    }
  }

    //Clears the setpoint logs 
    Logger.getInstance().recordOutput("SwerveStates/Setpoints", new double[] {});
    Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", new double[] {});
   
          // Calculate module setpoints
          var setpointTwist =
          new Pose2d()
              .log(
                  new Pose2d(
                      setpoint.vxMetersPerSecond * Constants.loopPeriodSecs,
                      setpoint.vyMetersPerSecond * Constants.loopPeriodSecs,
                      new Rotation2d(setpoint.omegaRadiansPerSecond * Constants.loopPeriodSecs)));
      var adjustedSpeeds =
          new ChassisSpeeds(
              setpointTwist.dx / Constants.loopPeriodSecs,
              setpointTwist.dy / Constants.loopPeriodSecs,
              setpointTwist.dtheta / Constants.loopPeriodSecs);
      SwerveModuleState[] setpointStates = swerveKinematics.toSwerveModuleStates(adjustedSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DrivetrainConstants.maxLinearSpeed);

      // Send setpoints to modules
      SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
      for (int i = 0; i < 4; i++) {
        optimizedStates[i] = modules[i].runSetpoint(setpointStates[i]);
      }

      // Log setpoint states
      Logger.getInstance().recordOutput("SwerveStates/Setpoints", setpointStates);
      Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", optimizedStates);

          // Log measured states
    SwerveModuleState[] measuredStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      measuredStates[i] = modules[i].getState();
    }
    Logger.getInstance().recordOutput("SwerveStates/Measured", measuredStates);
    // Update odometry
    SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      wheelDeltas[i] =
          new SwerveModulePosition(
              (modules[i].getPositionMeters() - lastModulePositionsMeters[i]),
              modules[i].getAngle());
      lastModulePositionsMeters[i] = modules[i].getPositionMeters();
    }

    var twist = swerveKinematics.toTwist2d(wheelDeltas);
    var gyroYaw = new Rotation2d(gyro.getYaw().getRadians());
    if (gyro.isConnected()) {
      twist = new Twist2d(twist.dx, twist.dy, gyroYaw.minus(lastGyroYaw).getRadians());
    }
    lastGyroYaw = gyroYaw;
    // poseEstimator.addDriveData(Timer.getFPGATimestamp(), twist);
    // Logger.getInstance().recordOutput("Odometry/Robot", getPose());

    // Update field velocity
    ChassisSpeeds chassisSpeeds = swerveKinematics.toChassisSpeeds(measuredStates);
    Translation2d linearFieldVelocity =
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
            .rotateBy(gyro.getYaw());  // TODO: change this to pose estimator get rotation 
    fieldVelocity =
        new Twist2d(
            linearFieldVelocity.getX(),
            linearFieldVelocity.getY(),
            gyro.isConnected()
                ? gyro.getYawVelocity()
                : chassisSpeeds.omegaRadiansPerSecond);
    
  }

  // TODO:continue the periodic

  public void runVelocity(ChassisSpeeds speeds) {
    DrivetrainConstants.ischaracterizing = false;
    setpoint = speeds;
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

  public Twist2d getFieldVelocity() {
    return fieldVelocity;
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

}
