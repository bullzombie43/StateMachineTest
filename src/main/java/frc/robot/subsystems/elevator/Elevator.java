// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.forwardSoftLimitMeters;
import static frc.robot.subsystems.elevator.ElevatorConstants.reverseSoftLimitMeters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

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
    IN_TRANSITION,
    AT_TARGET
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
  private SystemState systemState = SystemState.AT_TARGET;
  private WantedState prevWantedState = WantedState.STOW;

  private double setpointMeters = 0.0;
  private boolean atSetpoint = atSetpoint();

  private boolean travellingUpward = true;
  private double previousCarriageHeight = 0.0;
  private double carriageHeight = 0.0;
  private double stageHeight = 0.0;

  public Elevator(ElevatorIO io) {
    this.elevatorIO = io;
    elevatorIO.setGains(kP.get(), kA.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get());
    elevatorIO.setProfileConstraints(maxVeloRotPerSecTunable.get(), maxAccelRotPerSecTunable.get());
  }

  @Override
  public void periodic() {
    // Process Inputs
    elevatorIO.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // Update if we are at the setpoint each loop so behavior is consistent within each loop
    atSetpoint = atSetpoint();
    Logger.recordOutput("Elevator/atSetpoint", atSetpoint);

    // Set Alerts
    leftMotorDisconnected.set(inputs.leftMotorConnected);
    rightMotorDisconnected.set(inputs.rightMotorConnected);

    // Update Gains and Constraints
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            elevatorIO.setGains(
                kP.get(), kA.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get()),
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

    /*
     * State Machine Logic
     */

    // Update the intended SystemState based on the desired WantedState
    SystemState newState = handleStateTransition();
    if (newState != systemState) {
      Logger.recordOutput("Elevator/SystemState", newState.toString());
      systemState = newState;
    }

    // If we're disabled we are forced to IDLE
    if (DriverStation.isDisabled()) {
      systemState = SystemState.IS_IDLE;
    }

    // Control the motors based on the system state
    if (systemState == SystemState.IS_IDLE) {
      handleIdling();
    } else {
      switch (wantedState) {
        case STOW:
          handleStow();
          break;
        case INTAKE:
          handleIntake();
        case L1:
          handleL1();
          break;
        case L2:
          handleL2();
          break;
        case L3:
          handleL3();
          break;
        case L4:
          handleL4();
          break;
        case BARGE:
          handleBarge();
          break;
        case PROCESSOR:
          handleProcessor();
          break;
        case LOW_ALGEA:
          handleLowAlgea();
          break;
        case HIGH_ALGEA:
          handleHighAlgea();
          break;
        case CLIMB:
          handleClimb();
          break;
        case IDLE:
        default:
          break;
      }
    }

    // Log Outputs
    Logger.recordOutput("Elevator/WantedState", wantedState);
    Logger.recordOutput("Elevator/setpointMeters", setpointMeters);
    Logger.recordOutput("Elevator/setpointRotations", metersToRotations(setpointMeters));
    Logger.recordOutput("Elevator/currentHeight", getCurrentPositionMeters());

    // Update the pose3d visualization
    updateVisualization();
  }

  public SystemState handleStateTransition() {
    switch (wantedState) {
      case IDLE:
        return SystemState.IS_IDLE;
      default:
        // If we are not at the setpoint / goal state, then we are in transition
        return atSetpoint ? SystemState.AT_TARGET : SystemState.IN_TRANSITION;
    }
  }

  /*Exposed Methods for setting and getting state and setpoint */
  private void setWantedStateFunc(WantedState newWantedState) {
    this.prevWantedState = this.wantedState;
    this.wantedState = newWantedState;
  }

  public void setSetpointMeters(double targetHeightMeters) {
    double clampedMeters =
        MathUtil.clamp(targetHeightMeters, forwardSoftLimitMeters, reverseSoftLimitMeters);
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

  public double getCurrentPositionMeters() {
    return inputs.elevatorHeightMeters;
  }

  public double getVerticalVelocity() {
    return inputs.elevatorVelocityMetersPerSec;
  }

  public boolean elevatorAtSetpoint() {
    return MathUtil.isNear(
        setpointMeters, inputs.elevatorHeightMeters, ElevatorConstants.elevatorToleranceMeters);
  }

  public boolean atSetpoint() {
    return MathUtil.isNear(
        setpointMeters, inputs.elevatorHeightMeters, ElevatorConstants.elevatorToleranceMeters);
  }

  /*
   *
   * Handle Methods
   *
   */

  public void handleIdling() {
    elevatorIO.setVoltage(0.0, 0.0);
  }

  public void handleStow() {
    setSetpointMeters(ElevatorConstants.stowHeight);
    elevatorIO.setSetpointMeters(setpointMeters);
  }

  public void handleIntake() {
    setSetpointMeters(ElevatorConstants.intakeHeight);
    elevatorIO.setSetpointMeters(setpointMeters);
  }

  public void handleL1() {
    setSetpointMeters(ElevatorConstants.L1Height);
    elevatorIO.setSetpointMeters(setpointMeters);
  }

  public void handleL2() {
    setSetpointMeters(ElevatorConstants.L2Height);
    elevatorIO.setSetpointMeters(setpointMeters);
  }

  public void handleL3() {
    setSetpointMeters(ElevatorConstants.L3Height);
    elevatorIO.setSetpointMeters(setpointMeters);
  }

  public void handleL4() {
    setSetpointMeters(ElevatorConstants.L4Height);
    elevatorIO.setSetpointMeters(setpointMeters);
  }

  public void handleBarge() {
    setSetpointMeters(ElevatorConstants.bargeHeight);
    elevatorIO.setSetpointMeters(setpointMeters);
  }

  public void handleProcessor() {
    setSetpointMeters(ElevatorConstants.processorHeight);
    elevatorIO.setSetpointMeters(setpointMeters);
  }

  public void handleLowAlgea() {
    setSetpointMeters(ElevatorConstants.lowAlgeaHeight);
    elevatorIO.setSetpointMeters(setpointMeters);
  }

  public void handleHighAlgea() {
    setSetpointMeters(ElevatorConstants.highAlgeaHeight);
    elevatorIO.setSetpointMeters(setpointMeters);
  }

  public void handleClimb() {
    setSetpointMeters(ElevatorConstants.climbHeight);
    elevatorIO.setSetpointMeters(setpointMeters);
  }

  /*
   *
   * Commands
   *
   */

  public Command setWantedState(WantedState newWantedState) {
    return Commands.runOnce(() -> setWantedStateFunc(newWantedState), this);
  }

  /*Visualization Stuff */
  public void updateVisualization() {
    carriageHeight = getCurrentPositionMeters();
    boolean travellingUpwards = carriageHeight > previousCarriageHeight;

    if (travellingUpwards) {
      double stageTop = stageHeight + ElevatorConstants.stage1Length;

      if (carriageHeight < stageTop) {
        // Carriage is moving up the stage — do nothing
      } else {
        // Carriage is maxed out — move the stage up
        stageHeight = carriageHeight - ElevatorConstants.stage1Length;
      }

    } else {
      if (carriageHeight > stageHeight) {
        // Carriage is riding down the stage — do nothing
      } else {
        // Carriage has hit bottom of stage — retract stage
        stageHeight = carriageHeight;
      }
    }

    previousCarriageHeight = carriageHeight;

    Pose3d stage1Pose =
        new Pose3d(
            ElevatorConstants.stage1XOffset,
            ElevatorConstants.stage1YOffset,
            ElevatorConstants.stage1ZOffset + stageHeight,
            new Rotation3d());

    Pose3d carriagePose = new Pose3d(0, 0, carriageHeight, new Rotation3d());

    Robot.componentPoses[1] = stage1Pose;
    Robot.componentPoses[2] = carriagePose;

    // Set the Z position of the pivot in the component poses
    Robot.componentPoses[3] =
        new Pose3d(
            PivotConstants.pivotOffsetX,
            PivotConstants.pivotOffsetY,
            PivotConstants.pivotOffsetZ + carriageHeight,
            Robot.componentPoses[3].getRotation());
  }

  public static double rotationsToMeters(double rotations) {
    return rotations * ElevatorConstants.ROTATIONS_TO_METERS;
  }

  public static double metersToRotations(double meters) {
    return meters / ElevatorConstants.ROTATIONS_TO_METERS;
  }
}
