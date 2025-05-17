// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.forwardSoftLimitMeters;
import static frc.robot.subsystems.elevator.ElevatorConstants.reverseSoftLimitMeters;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class Elevator extends SubsystemBase {
  public enum WantedState {
    IDLE,
    STOW,
    INTAKE,
    L1,
    L2,
    L3,
    L4,
    BARGE,
    PROCESSOR,
    LOW_ALGEA,
    HIGH_ALGEA,
    CLIMB
  }

  public enum SystemState {
    IS_IDLE,
    STOWING,
    INTAKING,
    NORMAL_TARGETTING,
    CLIMBING
  }

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Elevator/kP", ElevatorConstants.gains.kP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Elevator/kI", ElevatorConstants.gains.kI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Elevator/kD", ElevatorConstants.gains.kD);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Elevator/kS", ElevatorConstants.gains.kS);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Elevator/kV", ElevatorConstants.gains.kV);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Elevator/kA", ElevatorConstants.gains.kA);
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Elevator/kG", ElevatorConstants.gains.kG);
  private static final LoggedTunableNumber maxVeloRotPerSecTunable =
      new LoggedTunableNumber(
          "Elevator/MaxVelocityRotPerSec", ElevatorConstants.maxVelocityRotPerSec);
  private static final LoggedTunableNumber maxAccelRotPerSecTunable =
      new LoggedTunableNumber(
          "Elevator/MaxAccelerationRotPerSec", ElevatorConstants.maxAccelerationRotPerSec);

  private final Alert leftMotorDisconnected =
      new Alert("Left Motor Disconnected", Alert.AlertType.kWarning);
  private final Alert rightMotorDisconnected =
      new Alert("Right Motor Disconnected", Alert.AlertType.kWarning);

  private final ElevatorIO elevatorIO;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private WantedState wantedState = WantedState.STOW;
  private SystemState systemState = SystemState.STOWING;
  private double setpointMeters = 0.0;

  public Elevator(ElevatorIO io) {
    this.elevatorIO = io;
    elevatorIO.setGains(kP.get(), kA.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get());
    elevatorIO.setProfileConstraints(
        maxVeloRotPerSecTunable.get(), maxAccelRotPerSecTunable.get());
  }

  @Override
  public void periodic() {
    //Process Inputs
    elevatorIO.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    //Set Alerts
    leftMotorDisconnected.set(inputs.leftMotorConnected);
    rightMotorDisconnected.set(inputs.rightMotorConnected);

    //Update Gains and Constraints
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            elevatorIO.setGains(kP.get(), kA.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get()),
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
            elevatorIO.setProfileConstraints(
                maxVeloRotPerSecTunable.get(), maxAccelRotPerSecTunable.get()),
        maxVeloRotPerSecTunable,
        maxAccelRotPerSecTunable);

    /*State Machine Logic */

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
      case NORMAL_TARGETTING:
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
    Logger.recordOutput("Pivot/setpointDegrees", setpointMeters);

  }

  public void handleStowing(){
    setSetpointMeters(ElevatorConstants.stowHeight);
    elevatorIO.setSetpointMeters(setpointMeters);
  }

  public void handleIntaking(){
    setSetpointMeters(ElevatorConstants.intakeHeight);
    elevatorIO.setSetpointMeters(setpointMeters);
  }

  public void handleClimbing(){
    setSetpointMeters(ElevatorConstants.climbHeight);
    elevatorIO.setSetpointMeters(setpointMeters);
  }

  public void handleIdling(){
    elevatorIO.setVoltage(0.0, 0.0);
  }

  public void handleTargetting() {
    switch (wantedState) {
      case L1:
        setSetpointMeters(ElevatorConstants.L1Height);
        elevatorIO.setSetpointMeters(setpointMeters);
        break;
      case L2:
        setSetpointMeters(ElevatorConstants.L2Height);
        elevatorIO.setSetpointMeters(setpointMeters);
        break;
      case L3:
        setSetpointMeters(ElevatorConstants.L3Height);
        elevatorIO.setSetpointMeters(setpointMeters);
        break;
      case L4:
        setSetpointMeters(ElevatorConstants.L4Height);
        elevatorIO.setSetpointMeters(setpointMeters);
        break;
      case BARGE:
        setSetpointMeters(ElevatorConstants.bargeHeight);
        elevatorIO.setSetpointMeters(setpointMeters);
        break;
      case PROCESSOR:
        setSetpointMeters(ElevatorConstants.processorHeight);
        elevatorIO.setSetpointMeters(setpointMeters);
        break;
      case LOW_ALGEA:
        setSetpointMeters(ElevatorConstants.lowAlgeaHeight);
        elevatorIO.setSetpointMeters(setpointMeters);
        break;
      case HIGH_ALGEA:
        setSetpointMeters(ElevatorConstants.highAlgeaHeight);
        elevatorIO.setSetpointMeters(setpointMeters);
        break;    
      default:
        break;
    }
  }

  public SystemState handleStateTransition() {
    switch (wantedState) {
      case STOW:
        return SystemState.STOWING;
      case INTAKE:
        return SystemState.INTAKING;
      case L1:
        return SystemState.NORMAL_TARGETTING;
      case L2:  
        return SystemState.NORMAL_TARGETTING;
      case L3:
        return SystemState.NORMAL_TARGETTING;
      case L4:
        return SystemState.NORMAL_TARGETTING;
      case BARGE:
        return SystemState.NORMAL_TARGETTING;
      case PROCESSOR:
        return SystemState.NORMAL_TARGETTING;
      case LOW_ALGEA:
        return SystemState.NORMAL_TARGETTING;
      case HIGH_ALGEA:
        return SystemState.NORMAL_TARGETTING;
      case CLIMB:
        return SystemState.CLIMBING;
      case IDLE:
      default:
        return SystemState.IS_IDLE;
    }
  }

  /*Exposed Methods for setting and getting state and setpoint */
  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public void setSetpointMeters(double targetHeightMeters) {
    double clampedMeters =
        MathUtil.clamp(targetHeightMeters, reverseSoftLimitMeters, forwardSoftLimitMeters);
    this.setpointMeters = clampedMeters;
  }

  public WantedState getWantedState() {
    return wantedState;
  }

  public SystemState getSystemState() {
    return systemState;
  }

  public double getSetpointMeters() {
    return setpointMeters;
  }

  public double getCurrentPositionMeters(){
    return inputs.elevatorHeightMeters;
  }

  public boolean elevatorAtSetpoint(){
    return MathUtil.isNear(setpointMeters, inputs.elevatorHeightMeters, ElevatorConstants.elevatorToleranceMeters);
  }

}
