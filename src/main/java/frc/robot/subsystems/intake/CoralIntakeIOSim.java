// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;

/** Add your docs here. */
public class CoralIntakeIOSim implements CoralIntakeIO {
  private final IntakeSimulation intakeSimulation;
  private final AbstractDriveTrainSimulation drivetrainSimulation;
  private double setpointDegrees;
  private double velocityRPS;

  public CoralIntakeIOSim(AbstractDriveTrainSimulation driveTrainSimulation) {
    setpointDegrees = 0.0;

    this.drivetrainSimulation = driveTrainSimulation;

    this.intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "Coral",
            driveTrainSimulation,
            Meters.of(0.7),
            Meters.of(0.2),
            IntakeSimulation.IntakeSide.FRONT,
            1);
  }

  @Override
  public void updateInputs(CoralIntakeIO.CoralIntakeIOInputs inputs) {
    // Simulated inputs would be updated here
    inputs.pivotPositionDegrees = setpointDegrees; // Just set The pivot directly to the position
    inputs.pivotVelocityDegreesPerSec = 0.0; // Simulated velocity
    inputs.pivotVoltage = 0.0; // Simulated voltage
    inputs.pivotCurrent = 0.0; // Simulated current
    inputs.pivotTemperature = 30.0; // Simulated temperature
    inputs.pivotMotorConnected = true; // Assume motor is connected in simulation
    inputs.rollerVoltage = 0.0; // Simulated roller voltage
    inputs.rollerCurrent = 0.0; // Simulated roller current
    inputs.rollerTemperature = 30.0; // Simulated roller temperature
    inputs.rollerVelocityRPS = velocityRPS; // Simulated roller velocity
    inputs.rollerMotorConnected = true; // Assume roller motor is connected in simulation
    inputs.hasCoral = hasCoral();
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
  public void setPivotVoltage(double voltage) {
    // In simulation, we might not need to do anything here
  }

  @Override
  public void setPivotCurrent(double current) {
    // In simulation, we might not need to do anything here
  }

  @Override
  public void setPivotGains(
      double kP, double kI, double kD, double kS, double kA, double kV, double kG) {

    IntakeConstants.coralPivotGains.updateGains(kP, kI, kD, kS, kV, kA, kG);
  }

  @Override
  public void runIntakeForward() {
    // Simulate running the intake forward
    velocityRPS =
        IntakeConstants.coralRollerForwardSpeed; // Set a positive velocity for intake forward
    intakeSimulation.startIntake();
  }

  @Override
  public void runIntakeReverse() {
    // Simulate running the intake in reverse
    velocityRPS =
        IntakeConstants.coralRollerReverseSpeed; // Set a negative velocity for intake reverse
    intakeSimulation.stopIntake();

    if (intakeSimulation.getGamePiecesAmount() > 0) {
      SimulatedArena.getInstance()
          .addGamePiece(
              new ReefscapeCoralOnField(
                  drivetrainSimulation
                      .getSimulatedDriveTrainPose()
                      .transformBy(new Transform2d(0.6, 0, new Rotation2d()))));
    }

    intakeSimulation.setGamePiecesCount(0);
  }

  @Override
  public void stopIntake() {
    // Simulate stopping the intake
    velocityRPS = 0.0; // Set velocity to zero to stop the intake
    intakeSimulation.stopIntake();
  }

  public boolean hasCoral() {
    return intakeSimulation.getGamePiecesAmount() > 0;
  }
}
