package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.drive.moduleIO;

import org.littletonrobotics.junction.Logger;

public class Module {
 private final moduleIO io;
 
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
 private final int index;

 private static final double wheelRadius = 1.75;
  private static final double driveKp = 0.0;
  private static final double driveKd = 0.0;
  private static final double driveKs = 0.2;
  private static final double driveKv = 0.2;
  private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
  private final PIDController driveFeedback =
      new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);
  private final PIDController turnFeedback =
      new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);
  //TODO;change to the real values

  Module(moduleIO io, int index) {
    System.out.println("[Init] Creating Module " + Integer.toString(index));
    this.io = io;
    this.index = index;

    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
  }
  public void periodic(){
  io.updateInputs(inputs);
  }

  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    // Optimize state based on current angle
    var optimizedState = SwerveModuleState.optimize(state,getAngle());

    // Run turn controller
    io.setTurnVoltage(
        turnFeedback.calculate(getAngle().getRadians(), optimizedState.angle.getRadians()));

    // Update velocity based on turn error
    optimizedState.speedMetersPerSecond *= Math.cos(turnFeedback.getPositionError());

    // Run drive controller
    double velocityRadPerSec = optimizedState.speedMetersPerSecond / Constants.WHEEL_RADIUS_METERS;
    io.setDriveVoltage(
        driveFeedforward.calculate(velocityRadPerSec)
            + driveFeedback.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec));

    return optimizedState;
  }
    /** Sets whether brake mode is enabled. */
    public void setBrakeMode(boolean enabled) {
      io.setDriveBrakeMode(enabled);
      io.setTurnBrakeMode(enabled);
    }

    public Rotation2d getAngle() {
      return new Rotation2d(MathUtil.angleModulus(inputs.turnAbsolutePositionRad));
    }
  //TODO:continue this
   
}

