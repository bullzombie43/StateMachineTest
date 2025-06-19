// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffector;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Robot;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.IntakeConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.ironmaple.utils.FieldMirroringUtils;

/** Add your docs here. */
public class EndEffectorIOSim implements EndEffectorIO {
  private final AbstractDriveTrainSimulation drivetrainSimulation;
  private double velocityRPS;
  private boolean hasCoral;
  private boolean hasAlgea;

  public EndEffectorIOSim(AbstractDriveTrainSimulation driveTrainSimulation) {
    this.drivetrainSimulation = driveTrainSimulation;
    this.velocityRPS = 0.0;
    this.hasCoral = false;
    this.hasAlgea = false;
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
    inputs.hasAlgea = hasAlgea; // Set the hasAlgea flag
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
    velocityRPS = EndEffectorConstants.coralOuttakeVel; // Set the velocity to forward speed

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
    velocityRPS = EndEffectorConstants.coralIntakeVel; // Set the velocity to reverse speed
  }

  @Override
  public void intakeAlgea() {
    velocityRPS = EndEffectorConstants.algeaIntakeVel; // Set the velocity to algea intake speed
  }

  @Override
  public void outtakeAlgea() {
    velocityRPS = EndEffectorConstants.algeaOuttakeVel; // Set the velocity to algea intake speed
  }

  @Override
  public void shootAlgea() {
    velocityRPS = EndEffectorConstants.algeaShootVel;

    if (Robot.robotContainer.getAlgeaIntake().hasAlgea()) {
      SimulatedArena.getInstance()
          .addGamePieceProjectile(
              new GamePieceProjectile(
                      ReefscapeAlgaeOnField.REEFSCAPE_ALGAE_INFO,
                      drivetrainSimulation.getSimulatedDriveTrainPose().getTranslation(),
                      Robot.componentPoses[3]
                          .transformBy(IntakeConstants.algeaIntakeOffset)
                          .getTranslation()
                          .toTranslation2d(),
                      drivetrainSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                      drivetrainSimulation.getSimulatedDriveTrainPose().getRotation(),
                      Meters.of(
                          Robot.componentPoses[3]
                              .transformBy(IntakeConstants.algeaIntakeOffset)
                              .getZ()),
                      MetersPerSecond.of(2.0),
                      Radians.of(
                          -Robot.componentPoses[3]
                              .transformBy(IntakeConstants.algeaIntakeOffset)
                              .getRotation()
                              .getY()))
                  .withTargetPosition(
                      () ->
                          FieldMirroringUtils.toCurrentAllianceTranslation(
                              new Translation3d(8.785, 6.146, 2.1)))
                  .withTargetTolerance(new Translation3d(0.8, 3, 0.1))
                  .withHitTargetCallBack(() -> Robot.addAlgeaToNet()));

      Robot.robotContainer.getAlgeaIntake().setHasAlgea(false);
    }
  }

  @Override
  public void stopEndEffector() {
    velocityRPS = 0.0; // Stop the end effector
  }
}
