// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

/** Add your docs here. */
public class CoralPivotIOSim implements CoralPivotIO {
  private double setpointDegrees;

  public CoralPivotIOSim() {
    setpointDegrees = 0.0;
  }

  @Override
  public void updateInputs(CoralPivotIO.CoralPivotIOInputs inputs) {
    // Simulated inputs would be updated here
    inputs.pivotPositionDegrees = setpointDegrees; // Just set The pivot directly to the position
    inputs.pivotVelocityDegreesPerSec = 0.0; // Simulated velocity
    inputs.pivotVoltage = 0.0; // Simulated voltage
    inputs.pivotCurrent = 0.0; // Simulated current
    inputs.pivotTemperature = 30.0; // Simulated temperature
    inputs.motorConnected = true; // Assume motor is connected in simulation
  }

  @Override
  public void setSetpointDegrees(double setpointInDegrees) {
    this.setpointDegrees = setpointInDegrees;
  }

  @Override
  public void setPositionDegrees(double position) {
    this.setpointDegrees = position; // Directly set the position in simulation
  }

  @Override
  public void setVoltage(double voltage) {
    // In simulation, we might not need to do anything here
  }

  @Override
  public void setCurrent(double current) {
    // In simulation, we might not need to do anything here
  }

  @Override
  public void setGains(
      double kP, double kI, double kD, double kS, double kA, double kV, double kG) {

    IntakeConstants.coralPivotGains.updateGains(kP, kI, kD, kS, kV, kA, kG);
  }
}
