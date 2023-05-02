package frc.robot.subsystems.ElectricalBoardMotor;

import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {
  @AutoLog
  public static class MotorIOInputs {
    public double motorPositionRad = 0.0;
    public double motorVelocityRadPerSec = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(MotorIOInputs inputs) {

  }

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double motorVoltage) {

  }
}
