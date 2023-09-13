// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.subsystems.gyro;


import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface GyroIO {
  
  @AutoLog
  public static class GyroIOInputs implements LoggableInputs {
    public boolean connected = false;
    public double rollPositionRad = 0.0;
    public double pitchPositionRad = 0.0;
    public double yawPositionRad = 0.0;
    public double anglePositionRad = 0.0;
    public double rate = 0.0;
    public double rollVelocityRadPerSec = 0.0;
    public double pitchVelocityRadPerSec = 0.0;
    public double yawVelocityRadPerSec = 0.0;
    public double heading = 0.0;

    @Override
    public void toLog(LogTable table) {
        // TODO Auto-generated method stub
        
    }
    @Override
    public void fromLog(LogTable table) {
        // TODO Auto-generated method stub
        
    }
  }

  public default void updateInputs(GyroIOInputs inputs) {}


  public default void updateHeading() {
    
  }
}

