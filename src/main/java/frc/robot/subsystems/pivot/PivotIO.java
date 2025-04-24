package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  default void setSetpointDegrees(double setpointInDegrees) {}

  default void setZeroPosition(double position) {}

  default void updateInputs(PivotIOInputs inputs) {}

  default void setVoltage(double voltage) {}

  default void setCurrent(double current) {}

  default void setGains(
      double kP, double kI, double kD, double kS, double kA, double kV, double kG) {}

  default void setProfileConstraints(double maxVelocity, double maxAcceleration) {}

  @AutoLog
  class PivotIOInputs {
    public boolean motorConnected = true;
    public boolean encoderConnected = true;
    public double pivotVoltage = 0.0;
    public double pivotCurrent = 0.0;
    public double pivotTemperature = 0.0;
    public double pivotPositionDegrees = 0.0;
    public double pivotVelocityDegreesPerSec = 0.0;
    public double absolutePositionDegrees = 0.0;
    public double absoluteVelocityDegreesPerSec = 0.0;
  }
}
