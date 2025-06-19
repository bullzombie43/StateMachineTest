// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffector;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  public enum WantedState {
    IDLE,
    STOPPED,
    OUTTAKE,
    SHOOT_ALGEA,
    INTAKE
  }

  public enum SystemState {
    IS_IDLE,
    STOPPING,
    OUTTAKING,
    SHOOTING_ALGEA,
    INTAKING
  }

  private final EndEffectorIO endEffectorIO;
  private final EndEffectorIOInputsAutoLogged endEffectorIOInputs =
      new EndEffectorIOInputsAutoLogged();

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IS_IDLE;
  private SystemState prevSystemState = SystemState.IS_IDLE;

  /** Creates a new EndEffector. */
  public EndEffector(EndEffectorIO endEffectorIO) {
    this.endEffectorIO = endEffectorIO;
  }

  @Override
  public void periodic() {
    // Process Inputs
    endEffectorIO.updateInputs(endEffectorIOInputs);
    Logger.processInputs("EndEffector", endEffectorIOInputs);

    /*
     * State Machine Logic
     */

    SystemState newState = handleStateTransition();
    if (newState != systemState) {
      Logger.recordOutput("EndEffector/SystemState", newState.toString());
      systemState = newState;
    }

    // If we're disabled we are forced to IDLE
    if (DriverStation.isDisabled()) {
      systemState = SystemState.IS_IDLE;
    }

    // Control motors based on SystemState
    if (systemState == SystemState.IS_IDLE) {
      handleIdling();
    } else {
      switch (wantedState) {
        case INTAKE:
          handleIntaking();
          break;
        case OUTTAKE:
          handleOuttaking();
          break;
        case STOPPED:
          handleStopping();
          break;
        case SHOOT_ALGEA:
          handleShootAlgea();
          break;
        case IDLE:
          handleIdling();
          break;
      }

      // Log Outputs
      Logger.recordOutput("EndEffector/WantedState", wantedState);
    }
  }

  public SystemState handleStateTransition() {
    switch (wantedState) {
      case STOPPED:
        return SystemState.STOPPING;
      case OUTTAKE:
        return SystemState.OUTTAKING;
      case INTAKE:
        return SystemState.INTAKING;
      case SHOOT_ALGEA:
        return SystemState.SHOOTING_ALGEA;
      case IDLE:
      default:
        return SystemState.IS_IDLE;
    }
  }

  public void handleIdling() {
    endEffectorIO.stopEndEffector();
  }

  public void handleIntaking() {
    endEffectorIO.intakeCoral();
  }

  public void handleOuttaking() {
    endEffectorIO.outtakeCoral();
  }

  public void handleStopping() {
    endEffectorIO.stopEndEffector();
  }

  public void handleShootAlgea() {
    endEffectorIO.shootAlgea();
  }

  public void setWantedStateFunc(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public WantedState getWantedState() {
    return wantedState;
  }

  public SystemState getSystemState() {
    return systemState;
  }

  public boolean hasCoral() {
    return endEffectorIOInputs.hasCoral;
  }

  public boolean hasAlgea() {
    return endEffectorIOInputs.hasAlgea;
  }
}
