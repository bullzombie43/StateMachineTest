// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class ElevatorIOSim extends ElevatorIOPhoenix6 {
  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(2),
          ElevatorConstants.elevatorGearRatio,
          ElevatorConstants.carriageMassKg,
          ElevatorConstants.drumRadius,
          ElevatorConstants.forwardSoftLimitMeters,
          ElevatorConstants.reverseSoftLimitMeters,
          true,
          0);

  private Notifier simNotifier = null;

  private double lastUpdateTimestamp;

  public ElevatorIOSim() {
    simNotifier = new Notifier(() -> updateSimState());
    simNotifier.startPeriodic(
        0.005); // Run Simulation at a faster rate so PID gains perform more reasonably

    rightMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;
    leftMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    super.updateInputs(inputs);
  }

  public void updateSimState() {
    var leftMotorSimState = leftMotor.getSimState();
    var rightMotorSimState = rightMotor.getSimState();

    leftMotorSimState.setSupplyVoltage(12.0);
    rightMotorSimState.setSupplyVoltage(12.0);

    double motorSimVoltage = rightMotorSimState.getMotorVoltage();
    elevatorSim.setInputVoltage(motorSimVoltage);
    Logger.recordOutput("Elevator/Sim/SimVoltage", motorSimVoltage);

    double timestamp = Timer.getTimestamp();
    elevatorSim.update(timestamp - lastUpdateTimestamp);
    lastUpdateTimestamp = timestamp;

    double simPositionMeters = elevatorSim.getPositionMeters();
    double simVelocityMetersPerSecond = elevatorSim.getVelocityMetersPerSecond();
    Logger.recordOutput("Elevator/Sim/SimPositionMeters", simPositionMeters);
    Logger.recordOutput("Elevator/Sim/SimVelocityMetersPerSecond", simVelocityMetersPerSecond);

    // Mutate Motor Position and Velocity
    double rotorPosition = Elevator.metersToRotations(simPositionMeters);
    leftMotorSimState.setRawRotorPosition(rotorPosition * ElevatorConstants.elevatorGearRatio);
    rightMotorSimState.setRawRotorPosition(rotorPosition * ElevatorConstants.elevatorGearRatio);
    Logger.recordOutput("Elevator/Sim/RawRotorPosition", rotorPosition * ElevatorConstants.elevatorGearRatio);

    double rotorVelocity = Elevator.metersToRotations(simVelocityMetersPerSecond);
    leftMotorSimState.setRotorVelocity(rotorVelocity * ElevatorConstants.elevatorGearRatio);
    rightMotorSimState.setRotorVelocity(rotorVelocity * ElevatorConstants.elevatorGearRatio);
    Logger.recordOutput("Elevator/Sim/RawRotorVelocity", rotorVelocity * ElevatorConstants.elevatorGearRatio);
  }
}
