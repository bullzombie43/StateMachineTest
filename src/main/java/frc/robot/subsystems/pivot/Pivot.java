// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotConstants.acceptablePitchErrorDegrees;
import static frc.robot.subsystems.pivot.PivotConstants.climbDegrees;
import static frc.robot.subsystems.pivot.PivotConstants.forwardSoftLimitDegrees;
import static frc.robot.subsystems.pivot.PivotConstants.intakeDegrees;
import static frc.robot.subsystems.pivot.PivotConstants.reverseSoftLimitDegrees;
import static frc.robot.subsystems.pivot.PivotConstants.stowDegrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  public enum WantedState {
    IDLE,
    STOW,
    INTAKE,
    AT_TARGET,
    CLIMB
  }

  public enum SystemState {
    IS_IDLE,
    STOWING,
    INTAKING,
    MOVING_TO_TARGET,
    CLIMBING
  }

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Pivot/kP", PivotConstants.gains.kP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Pivot/kI", PivotConstants.gains.kI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Pivot/kD", PivotConstants.gains.kD);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Pivot/kS", PivotConstants.gains.kS);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Pivot/kV", PivotConstants.gains.kV);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Pivot/kA", PivotConstants.gains.kA);
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Pivot/kG", PivotConstants.gains.kG);
  private static final LoggedTunableNumber maxVeloRotPerSecTunable =
      new LoggedTunableNumber("Pivot/MaxVelocityRotPerSec", PivotConstants.maxVelocityRotPerSec);
  private static final LoggedTunableNumber maxAccelRotPerSecTunable =
      new LoggedTunableNumber(
          "Pivot/MaxAccelerationRotPerSec", PivotConstants.maxAccelerationRotPerSec);

  private final Alert pivotMotorDisconnected =
      new Alert("Pivot Motor Disconnected", Alert.AlertType.kWarning);
  private final Alert pivotEncoderDisconnected =
      new Alert("Pivot Encoder Disconnected", Alert.AlertType.kWarning);

  private final PivotIO pivotIO;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private WantedState wantedState = WantedState.STOW;
  private SystemState systemState = SystemState.STOWING;
  private double setpointDegrees = 0.0;

  /** Creates a new Pivot. */
  public Pivot(PivotIO io) {
    this.pivotIO = io;
    pivotIO.setGains(kP.get(), kA.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get());
    pivotIO.setProfileConstraints(maxVeloRotPerSecTunable.get(), maxAccelRotPerSecTunable.get());
  }

  @Override
  public void periodic() {
    // Process Inputs
    pivotIO.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);

    // Set Alerts
    pivotMotorDisconnected.set(!inputs.motorConnected);
    pivotEncoderDisconnected.set(!inputs.encoderConnected);

    // Update Gains and Motion Constraints
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            pivotIO.setGains(kP.get(), kA.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get()),
        kP,
        kI,
        kD,
        kS,
        kV,
        kA,
        kG);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            pivotIO.setProfileConstraints(
                maxVeloRotPerSecTunable.get(), maxAccelRotPerSecTunable.get()),
        maxVeloRotPerSecTunable,
        maxAccelRotPerSecTunable);

    /*State Machine Logic*/

    // Update the intended SystemState based on the desired WantedState
    SystemState newState = handleStateTransition();
    if (newState != systemState) {
      Logger.recordOutput("Pivot/SystemState", newState.toString());
      systemState = newState;
    }

    // If we're disabled we are forced to IDLE
    if (DriverStation.isDisabled()) {
      systemState = SystemState.IS_IDLE;
    }

    // Control the motors based on the system state
    switch (systemState) {
      case STOWING:
        handleStowing();
        break;
      case INTAKING:
        handleIntaking();
        break;
      case MOVING_TO_TARGET:
        handleTargetting();
        break;
      case CLIMBING:
        handleClimbing();
        break;
      case IS_IDLE:
      default:
        handleIdling();
    }

    // Log Outputs
    Logger.recordOutput("Pivot/WantedState", wantedState);
    Logger.recordOutput("Pivot/setpointDegrees", setpointDegrees);
  }

  public SystemState handleStateTransition() {
    switch (wantedState) {
      case STOW:
        return SystemState.STOWING;
      case INTAKE:
        return SystemState.INTAKING;
      case AT_TARGET:
        return SystemState.MOVING_TO_TARGET;
      case CLIMB:
        return SystemState.CLIMBING;
      case IDLE:
      default:
        return SystemState.IS_IDLE;
    }
  }

  /*Methods to handle each of the system states */
  public void handleStowing() {
    setSetpointDegrees(stowDegrees);
    pivotIO.setSetpointDegrees(setpointDegrees);
  }

  public void handleIntaking() {
    setSetpointDegrees(intakeDegrees);
    pivotIO.setSetpointDegrees(setpointDegrees);
  }

  public void handleTargetting() {
    pivotIO.setSetpointDegrees(setpointDegrees);
  }

  public void handleClimbing() {
    setSetpointDegrees(climbDegrees);
    pivotIO.setSetpointDegrees(setpointDegrees);
  }

  public void handleIdling() {
    pivotIO.setVoltage(0.0);
  }

  /*Exposed Methods for setting and getting state and setpoint */
  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public void setWantedState(WantedState wantedState, double setpointDegrees) {
    this.wantedState = wantedState;
    setSetpointDegrees(setpointDegrees);
  }

  public void setSetpointDegrees(double angleDegrees) {
    double clampedDegrees =
        MathUtil.clamp(angleDegrees, reverseSoftLimitDegrees, forwardSoftLimitDegrees);
    this.setpointDegrees = clampedDegrees;
  }

  public WantedState getWantedState() {
    return wantedState;
  }

  public SystemState getSystemState() {
    return systemState;
  }

  public boolean pivotAtSetpoint() {
    return MathUtil.isNear(
        setpointDegrees, inputs.pivotPositionDegrees, acceptablePitchErrorDegrees);
  }

  public double getCurrentPositionDegrees() {
    return inputs.pivotPositionDegrees;
  }
}
