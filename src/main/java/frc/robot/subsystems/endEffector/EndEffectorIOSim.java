// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffector;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import frc.robot.Robot;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.IntakeConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

/** Add your docs here. */
public class EndEffectorIOSim implements EndEffectorIO {
  private final AbstractDriveTrainSimulation drivetrainSimulation;
  private double velocityRPS;
  private boolean hasCoral;

  public EndEffectorIOSim(AbstractDriveTrainSimulation driveTrainSimulation) {
    this.drivetrainSimulation = driveTrainSimulation;
    this.velocityRPS = 0.0;
    this.hasCoral = false;
  }

  @Override
  public void updateInputs(EndEffectorIO.EndEffectorIOInputs inputs) {
    // Simulated inputs would be updated here
    inputs.endEffectorVoltage = 0.0; // Simulated voltage
    inputs.endEffectorCurrent = 0.0; // Simulated current
    inputs.endEffectorTemperature = 30.0; // Simulated temperature
    inputs.endEffectorVelocityRPS = velocityRPS; // Simulated velocity
    inputs.endEffectorMotorConnected = true; // Assume motor is connected in simulation
    inputs.hasCoral = hasCoral; // Set the hasCoral flag
  }

  @Override
  public void setVoltage(double voltage) {
    // Do Nothing
  }

  @Override
  public void setCurrent(double current) {
    // Do Nothing
  }

  @Override
  public void setGains(
      double kP, double kI, double kD, double kS, double kA, double kV, double kG) {
    // Do Nothing
    EndEffectorConstants.gains.updateGains(kP, kI, kD, kS, kV, kA, kG);
  }

  @Override
  public void outtakeCoral() {
    velocityRPS = EndEffectorConstants.forwardSpeed; // Set the velocity to forward speed

    if (Robot.robotContainer.getCoralIntake().hasCoral()) {
      SimulatedArena.getInstance()
          .addGamePieceProjectile(
              new ReefscapeCoralOnFly(
                  drivetrainSimulation.getSimulatedDriveTrainPose().getTranslation(),
                  Robot.componentPoses[3]
                      .transformBy(IntakeConstants.coralIntakeOffset)
                      .getTranslation()
                      .toTranslation2d(),
                  drivetrainSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                  drivetrainSimulation.getSimulatedDriveTrainPose().getRotation(),
                  Meters.of(Superstructure.coralPose.getTranslation().getZ()),
                  MetersPerSecond.of(2),
                  Degrees.of(
                      Robot.componentPoses[3]
                              .transformBy(IntakeConstants.coralIntakeOffset)
                              .getRotation()
                              .getMeasureY()
                              .in(Degrees)
                          - 90)));

      Robot.robotContainer.getCoralIntake().setHasCoral(hasCoral);
    }
  }

  @Override
  public void intakeCoral() {
    velocityRPS = EndEffectorConstants.reverseSpeed; // Set the velocity to reverse speed
  }

  @Override
  public void stopEndEffector() {
    velocityRPS = 0.0; // Stop the end effector
  }
}
