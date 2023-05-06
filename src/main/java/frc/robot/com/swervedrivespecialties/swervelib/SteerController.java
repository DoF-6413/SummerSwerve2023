package frc.robot.com.swervedrivespecialties.swervelib;
;

public interface SteerController {
    double getReferenceAngle();

    void setReferenceAngle(double referenceAngleRadians);

    double getStateAngle();
}
