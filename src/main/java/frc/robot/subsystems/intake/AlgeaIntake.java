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

public class AlgeaIntake extends SubsystemBase {
  public enum WantedState {
    IDLE,
    STOW,
    INTAKE,
    OUT_NO_INTAKE,
    OUTTAKE
  }

  public enum SystemState {
    IS_IDLE,
    STOWING,
    INTAKING,
    OUT_NO_INTAKE,
    OUTTAKING
  }

  private static final LoggedTunableNumber pivotKP =
      new LoggedTunableNumber("AlgeaIntake/pivot/kP", IntakeConstants.algeaPivotGains.kP);
  private static final LoggedTunableNumber pivotKI =
      new LoggedTunableNumber("AlgeaIntake/pivot/kI", IntakeConstants.algeaPivotGains.kI);
  private static final LoggedTunableNumber pivotKD =
      new LoggedTunableNumber("AlgeaIntake/pivot/kD", IntakeConstants.algeaPivotGains.kD);
  private static final LoggedTunableNumber pivotKS =
      new LoggedTunableNumber("AlgeaIntake/pivot/kS", IntakeConstants.algeaPivotGains.kS);
  private static final LoggedTunableNumber pivotKV =
      new LoggedTunableNumber("AlgeaIntake/pivot/kV", IntakeConstants.algeaPivotGains.kV);
  private static final LoggedTunableNumber pivotKA =
      new LoggedTunableNumber("AlgeaIntake/pivot/kA", IntakeConstants.algeaPivotGains.kA);
  private static final LoggedTunableNumber pivotKG =
      new LoggedTunableNumber("AlgeaIntake/pivot/kG", IntakeConstants.algeaPivotGains.kG);

  private final Alert pivotMotorDisconnected =
      new Alert(
          "Algea Intake Pivot Motor Disconnected",
          "The pivot motor is not connected.",
          Alert.AlertType.kWarning);

  private final AlgeaIntakeIO intakeIO;
  private final AlgeaIntakeIOInputsAutoLogged intakeIOInputs = new AlgeaIntakeIOInputsAutoLogged();

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IS_IDLE;
  private SystemState prevSystemState = SystemState.IS_IDLE;

  private double setpointDegrees = 0.0;
  private boolean pivotAtSetpoint = atPivotSetpoint();

  /** Creates a new AlgeaIntake. */
  public AlgeaIntake(AlgeaIntakeIO pivotIO) {
    this.intakeIO = pivotIO;
    this.intakeIO.setGains(
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
    Logger.processInputs("AlgeaIntake/Pivot", intakeIOInputs);

    // Update if we are at the setpoint each loop so behavior is consistent within
    // each loop
    pivotAtSetpoint = atPivotSetpoint();
    Logger.recordOutput("AlgeaIntake/Pivot/atSetpoint", pivotAtSetpoint);

    // Set Alerts
    pivotMotorDisconnected.set(!intakeIOInputs.motorConnected);

    // Update Gains
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            intakeIO.setGains(
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
      Logger.recordOutput("AlgeaIntake/SystemState", newState.toString());
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
        case OUTTAKE:
          handleOuttaking();
          break;
        case IDLE:
        default:
          break;
      }
    }

    // Log Outputs
    Logger.recordOutput("AlgeaIntake/WantedState", wantedState);
    Logger.recordOutput("AlgeaIntake/Pivot/setpointDegrees", setpointDegrees);
    Logger.recordOutput("AlgeaIntake/Pivot/CurrentPositionDegrees", getCurrentPositionDegrees());

    // Visualize the Pivot as a pose3d
    Robot.componentPoses[0] =
        new Pose3d(
            Units.inchesToMeters(-12.00),
            0,
            Units.inchesToMeters(7),
            new Rotation3d(
                Units.degreesToRadians(180),
                Units.degreesToRadians(getCurrentPositionDegrees()),
                Units.degreesToRadians(180)));
  }

  public SystemState handleStateTransition() {
    switch (wantedState) {
      case STOW:
        return SystemState.STOWING;
      case INTAKE:
        return SystemState.INTAKING;
      case OUT_NO_INTAKE:
        return SystemState.OUT_NO_INTAKE;
      case OUTTAKE:
        return SystemState.OUTTAKING;
      case IDLE:
      default:
        return SystemState.IS_IDLE;
    }
  }

  public void handleIdling() {
    intakeIO.setVoltage(0.0);
    intakeIO.stopRoller();
  }

  public void handleStowing() {
    setSetpointDegrees(IntakeConstants.algeaStowDegrees);
    intakeIO.setSetpointDegrees(setpointDegrees);
    intakeIO.stopRoller();
  }

  public void handleIntaking() {
    setSetpointDegrees(IntakeConstants.algeaIntakeDegrees);
    intakeIO.setSetpointDegrees(setpointDegrees);
    intakeIO.runRollerForward();
  }

  public void handleOutNoIntake() {
    setSetpointDegrees(IntakeConstants.algeaIntakeDegrees);
    intakeIO.setSetpointDegrees(setpointDegrees);
    intakeIO.stopRoller();
  }

  public void handleOuttaking() {
    setSetpointDegrees(IntakeConstants.algeaIntakeDegrees);
    intakeIO.setSetpointDegrees(setpointDegrees);
    intakeIO.runRollerReverse();
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

  public boolean hasAlgea() {
    return intakeIOInputs.hasAlgea;
  }
}
