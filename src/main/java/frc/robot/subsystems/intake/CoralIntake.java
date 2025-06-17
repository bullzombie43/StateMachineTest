// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class CoralIntake extends SubsystemBase {
  public enum WantedState {
    IDLE,
    STOW,
    INTAKE,
    REVERSING,
    OUT_NO_INTAKE
  }

  public enum SystemState {
    IS_IDLE,
    STOWING,
    INTAKING,
    REVERSING,
    OUT_NO_INTAKE
  }

  private static final LoggedTunableNumber pivotKP =
      new LoggedTunableNumber("CoralIntake/pivot/kP", IntakeConstants.coralPivotGains.kP);
  private static final LoggedTunableNumber pivotKI =
      new LoggedTunableNumber("CoralIntake/pivot/kI", IntakeConstants.coralPivotGains.kI);
  private static final LoggedTunableNumber pivotKD =
      new LoggedTunableNumber("CoralIntake/pivot/kD", IntakeConstants.coralPivotGains.kD);
  private static final LoggedTunableNumber pivotKS =
      new LoggedTunableNumber("CoralIntake/pivot/kS", IntakeConstants.coralPivotGains.kS);
  private static final LoggedTunableNumber pivotKV =
      new LoggedTunableNumber("CoralIntake/pivot/kV", IntakeConstants.coralPivotGains.kV);
  private static final LoggedTunableNumber pivotKA =
      new LoggedTunableNumber("CoralIntake/pivot/kA", IntakeConstants.coralPivotGains.kA);
  private static final LoggedTunableNumber pivotKG =
      new LoggedTunableNumber("CoralIntake/pivot/kG", IntakeConstants.coralPivotGains.kG);

  private final Alert pivotMotorDisconnected =
      new Alert(
          "Coral Intake Pivot Motor Disconnected",
          "The pivot motor is not connected.",
          Alert.AlertType.kWarning);

  private final CoralIntakeIO intakeIO;
  private final CoralIntakeIOInputsAutoLogged intakeIOInputs = new CoralIntakeIOInputsAutoLogged();

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IS_IDLE;
  private SystemState prevSystemState = SystemState.IS_IDLE;

  private double setpointDegrees = 0.0;
  private boolean pivotAtSetpoint = atPivotSetpoint();

  /** Creates a new CoralIntake. */
  public CoralIntake(CoralIntakeIO pivotIO) {
    this.intakeIO = pivotIO;
    this.intakeIO.setPivotGains(
        pivotKP.get(),
        pivotKI.get(),
        pivotKD.get(),
        pivotKS.get(),
        pivotKV.get(),
        pivotKA.get(),
        pivotKG.get());
  }

  @Override
  public void periodic() {
    // Process the pivot inputs
    intakeIO.updateInputs(intakeIOInputs);
    Logger.processInputs("CoralIntake/Pivot", intakeIOInputs);

    // Update if we are at the setpoint each loop so behavior is consistent within
    // each loop
    pivotAtSetpoint = atPivotSetpoint();
    Logger.recordOutput("CoralIntake/Pivot/atSetpoint", pivotAtSetpoint);

    // Set Alerts
    pivotMotorDisconnected.set(!intakeIOInputs.pivotMotorConnected);

    // Update Gains
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            intakeIO.setPivotGains(
                pivotKP.get(),
                pivotKI.get(),
                pivotKD.get(),
                pivotKS.get(),
                pivotKV.get(),
                pivotKA.get(),
                pivotKG.get()),
        pivotKP,
        pivotKI,
        pivotKD,
        pivotKS,
        pivotKV,
        pivotKA,
        pivotKG);

    /*
     * State Machine Logic
     */

    // Update the intended SystemState based on desired WantedState
    SystemState newState = handleStateTransition();
    if (newState != systemState) {
      Logger.recordOutput("CoralIntake/SystemState", newState.toString());
      systemState = newState;
    }

    // If we're disabled we are forced to IDLE
    if (DriverStation.isDisabled()) {
      systemState = SystemState.IS_IDLE;
    }

    // Control Motors Based om the SystemState
    if (systemState == SystemState.IS_IDLE) {
      handleIdling();
    } else {
      switch (wantedState) {
        case INTAKE:
          handleIntaking();
          break;
        case OUT_NO_INTAKE:
          handleOutNoIntake();
          break;
        case STOW:
          handleStowing();
          break;
        case REVERSING:
          handlReversing();
          break;
        case IDLE:
        default:
          break;
      }
    }

    // Log Outputs
    Logger.recordOutput("CoralIntake/WantedState", wantedState);
    Logger.recordOutput("CoralIntake/Pivot/setpointDegrees", setpointDegrees);
    Logger.recordOutput("CoralIntake/Pivot/CurrentPositionDegrees", getCurrentPositionDegrees());

    // Visualize the Pivot as a pose3d
    Robot.componentPoses[4] =
        new Pose3d(
            0.14,
            0,
            0.23,
            new Rotation3d(0, Units.degreesToRadians(getCurrentPositionDegrees()), 0));
  }

  public SystemState handleStateTransition() {
    switch (wantedState) {
      case STOW:
        return SystemState.STOWING;
      case INTAKE:
        return SystemState.INTAKING;
      case OUT_NO_INTAKE:
        return SystemState.OUT_NO_INTAKE;
      case REVERSING:
        return SystemState.REVERSING;
      case IDLE:
      default:
        return SystemState.IS_IDLE;
    }
  }

  public void handleIdling() {
    intakeIO.setPivotVoltage(0.0);
    intakeIO.stopIntake();
  }

  public void handlReversing() {
    setSetpointDegrees(IntakeConstants.coralIntakeDegrees);
    intakeIO.setSetpointDegrees(setpointDegrees);
    intakeIO.runIntakeReverse();
  }

  public void handleStowing() {
    setSetpointDegrees(IntakeConstants.coralStowDegrees);
    intakeIO.setSetpointDegrees(setpointDegrees);
    intakeIO.stopIntake();
  }

  public void handleIntaking() {
    setSetpointDegrees(IntakeConstants.coralIntakeDegrees);
    intakeIO.setSetpointDegrees(setpointDegrees);
    intakeIO.runIntakeForward();
  }

  public void handleOutNoIntake() {
    setSetpointDegrees(IntakeConstants.coralIntakeDegrees);
    intakeIO.setSetpointDegrees(setpointDegrees);
    intakeIO.stopIntake();
  }

  public void setWantedStateFunc(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public Command setWantedState(WantedState wantedState) {
    return Commands.runOnce(() -> setWantedStateFunc(wantedState), this);
  }

  public WantedState getWantedState() {
    return wantedState;
  }

  public SystemState getSystemState() {
    return systemState;
  }

  public boolean atPivotSetpoint() {
    return MathUtil.isNear(
        setpointDegrees,
        intakeIOInputs.pivotPositionDegrees,
        IntakeConstants.acceptablePitchErrorDegrees);
  }

  public double getCurrentPositionDegrees() {
    return intakeIOInputs.pivotPositionDegrees;
  }

  public void setSetpointDegrees(double angleDegrees) {
    double clampedDegrees =
        MathUtil.clamp(
            angleDegrees, IntakeConstants.minAngleDegrees, IntakeConstants.maxAngleDegrees);
    this.setpointDegrees = clampedDegrees;
  }

  public boolean hasCoral() {
    return intakeIOInputs.hasCoral;
  }
}
