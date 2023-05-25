package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.gyro.GyroIO;
import frc.robot.subsystems.gyro.GyroIO.GyroIOInputs;

public class Drive extends SubsystemBase {

  private final GyroIO gyroIO;
  private final GyroIOInputs gyroIOInputs = new GyroIOInputs();
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private static final Module[] modules = new Module[4];

  private double maxAngularSpeed;
  private SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics();

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
  public Drive(GyroIO gyroIO, moduleIO flModuleIO, moduleIO frModuleIO, moduleIO blModuleIO, moduleIO brModuleIO) {

    this.gyroIO = gyroIO;

    System.out.println("[Init] Creating Drive");

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

    gyroIO.updateInputs(gyroIOInputs);
    Logger.getInstance().processInputs("Drive/Gyro", gyroIOInputs);
    for (var module : modules) {
      module.periodic();
    }

    Logger.getInstance().processInputs("Drive", inputs);

    // Update odometry and log the new pose
    odometry.update(new Rotation2d(-Math.toDegrees(gyroIOInputs.yawPositionRad)), getLeftPositionMeters(),
        getRightPositionMeters());
    Logger.getInstance().recordOutput("Odometry", getPose());
  }

  // TODO:continue the periodic
  /** Run open loop at the specified percentage. */
  public void drivePercent(double leftPercent, double rightPercent) {
    // io(leftPercent * 12.0, rightPercent * 12.0);
  }

  /** Run open loop based on stick positions. */
  public void driveArcade(double xSpeed, double zRotation) {
    var speeds = DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, true);
    // io.setVoltage(speeds.left * 12.0, speeds.right * 12.0);
  }

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

  public Twist2d

  /** Returns the current odometry pose in meters. */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

}
