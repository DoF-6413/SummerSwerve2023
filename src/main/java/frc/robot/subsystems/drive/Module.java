package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.drive.moduleIO;

import org.littletonrobotics.junction.Logger;

public class Module {
  private final moduleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
  private final PIDController driveFeedback = new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);
  private final PIDController turnFeedback = new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);
  // TODO;change to the real values

  Module(moduleIO io, int index) {
    System.out.println("[Init] Creating Module " + Integer.toString(index));
    this.io = io;
    this.index = index;

    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void periodic() {
    io.updateInputs(inputs);
    System.out.println("hiiiiiiiii");
    SmartDashboard.putString("HI", "HIII");
    driveFeedback.setPID((DrivetrainConstants.driveKp), (0.0), (DrivetrainConstants.driveKd));
    turnFeedback.setPID((DrivetrainConstants.turnKp), (0.0), (DrivetrainConstants.turnKd));
    driveFeedforward = new SimpleMotorFeedforward(DrivetrainConstants.driveKs, DrivetrainConstants.driveKv);
  }
  

  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    // Optimize state based on current angle
    var optimizedState = SwerveModuleState.optimize(state, getAngle());

    // Run turn controller
    io.setTurnVoltage(
        turnFeedback.calculate(getAngle().getRadians(), optimizedState.angle.getRadians()));

    // Update velocity based on turn error
    optimizedState.speedMetersPerSecond *= Math.cos(turnFeedback.getPositionError());

    // Run drive controller
    double velocityRadPerSec = optimizedState.speedMetersPerSecond / DrivetrainConstants.WHEEL_RADIUS_METERS;
    io.setDriveVoltage(
        driveFeedforward.calculate(velocityRadPerSec)
            + driveFeedback.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec));

    return optimizedState;
  }

  /**
   * Runs the module with the specified voltage while controlling to zero degrees. Must be called
   * periodically.
   */
  public void runCharacterization(double volts) {
    io.setTurnVoltage(turnFeedback.calculate(getAngle().getRadians(), 0.0));
    io.setDriveVoltage(volts);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setTurnVoltage(0.0);
    io.setDriveVoltage(0.0);
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
    io.setTurnBrakeMode(enabled);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return new Rotation2d(MathUtil.angleModulus(inputs.turnAbsolutePositionRad));
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * DrivetrainConstants.WHEEL_RADIUS_METERS;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * DrivetrainConstants.WHEEL_RADIUS_METERS;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }

  /** Returns the drive wheel radius. */
  public static double getWheelRadius() {
    return DrivetrainConstants.WHEEL_RADIUS_METERS;
  }
}
