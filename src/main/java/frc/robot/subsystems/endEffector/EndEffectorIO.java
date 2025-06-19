// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface EndEffectorIO {
  default void outtakeCoral() {}

  default void intakeCoral() {}

  default void stopEndEffector() {}

  default void intakeAlgea() {}

  default void outtakeAlgea() {}

  default void shootAlgea() {}

  default void setGains(
      double kP, double kI, double kD, double kS, double kA, double kV, double kG) {}

  default void updateInputs(EndEffectorIOInputs inputs) {}

  default void setVoltage(double voltage) {}

  default void setCurrent(double current) {}

  @AutoLog
  class EndEffectorIOInputs {
    public boolean endEffectorMotorConnected = true;
    public double endEffectorVoltage = 0.0;
    public double endEffectorCurrent = 0.0;
    public double endEffectorTemperature = 0.0;
    public double endEffectorVelocityRPS = 0.0;

    public boolean hasCoral = false; // Indicates if the Coral end effector is present
    public boolean hasAlgea = false; // Indicates if the Algea end effector is present
  }
}
