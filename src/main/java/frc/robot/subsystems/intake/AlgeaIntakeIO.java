// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface AlgeaIntakeIO {
  default void setSetpointDegrees(double setpointInDegrees) {}

  default void setPositionDegrees(double position) {}

  default void updateInputs(AlgeaIntakeIOInputs inputs) {}

  default void setVoltage(double voltage) {}

  default void setCurrent(double current) {}

  default void setGains(
      double kP, double kI, double kD, double kS, double kA, double kV, double kG) {}

  default void runRollerForward() {}

  default void runRollerReverse() {}

  default void stopRoller() {}

  default void setHasAlgea(boolean hasAlgea) {
    // Only Used in simulation
  }

  @AutoLog
  class AlgeaIntakeIOInputs {
    public boolean motorConnected = true;
    public double pivotVoltage = 0.0;
    public double pivotCurrent = 0.0;
    public double pivotTemperature = 0.0;
    public double pivotPositionDegrees = 0.0;
    public double pivotVelocityDegreesPerSec = 0.0;

    public boolean rollerMotorConnected = true;
    public double rollerVoltage = 0.0;
    public double rollerCurrent = 0.0;
    public double rollerTemperature = 0.0;
    public double rollerVelocityRPS = 0.0;
    public boolean hasAlgea = false;
  }
}
