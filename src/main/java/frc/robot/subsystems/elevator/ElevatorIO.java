// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {
  default void setSetpointMeters(double setpointInMeters) {}

  default void setPositionMeters(double positionMeters) {}

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setVoltage(double leftVoltage, double rightVoltage) {}

  default void setCurrent(double leftCurrent, double rightCurrent) {}

  default void setGains(
      double kP, double kI, double kD, double kS, double kA, double kV, double kG) {}

  default void setProfileConstraints(double maxVelocity, double maxAcceleration) {}

  @AutoLog
  class ElevatorIOInputs {
    public boolean leftMotorConnected = true;
    public boolean rightMotorConnected = true;
    public double leftVoltage = 0.0;
    public double rightVoltage = 0.0;
    public double leftCurrent = 0.0;
    public double rightCurrent = 0.0;
    public double leftTemperature = 0.0;
    public double elevatorHeightMeters = 0.0;
    public double elevatorVelocityMetersPerSec = 0.0;
  }
}
