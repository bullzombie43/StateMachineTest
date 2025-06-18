// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotConstants.acceptablePitchErrorDegrees;
import static frc.robot.subsystems.pivot.PivotConstants.forwardSoftLimitDegrees;
import static frc.robot.subsystems.pivot.PivotConstants.reverseSoftLimitDegrees;

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

public class Pivot extends SubsystemBase {
  public enum WantedState {
    IDLE,
    STOW,
    INTAKE,
    ALGEA_INTAKE,
    L1,
    L2,
    L3,
    L4,
    BARGE,
    PROCESSOR,
    LOW_ALGEA,
    HIGH_ALGEA,
    CLIMB,
    AlGEA_OUTTAKE,
    CORAL_OUTTAKE,
    VOLTAGE
  }

  public enum SystemState {
    IS_IDLE,
    IN_TRANSITION,
    AT_TARGET,
    VOLTAGE
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
  private SystemState systemState = SystemState.AT_TARGET;
  private WantedState prevWantedState = WantedState.STOW;

  private double setpointDegrees = 0.0;
  private boolean atSetpoint = atSetpoint();

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

    // Update Final Setpoint based on Wanted State

    // Update if we are at the setpoint each loop so behavior is consistent within each loop
    atSetpoint = atSetpoint();
    Logger.recordOutput("Pivot/atSetpoint", atSetpoint);

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

    /*
     * State Machine Logic
     */

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
    if (systemState == SystemState.IS_IDLE) {
      handleIdling();
    } else if (systemState == SystemState.VOLTAGE) {
      // Just here so we don't seek a setpoint when trying to control a voltage
    } else {
      switch (wantedState) {
        case STOW:
          handleStow();
          break;
        case INTAKE:
          handleIntake();
          break;
        case ALGEA_INTAKE:
          handleAlgeaIntake();
          break;
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
        case AlGEA_OUTTAKE:
          handleAlgeaOuttake();
          break;
        case CORAL_OUTTAKE:
          handleCoralOuttake();
          break;
        case IDLE:
        default:
          break;
      }
    }

    // Log Outputs
    Logger.recordOutput("Pivot/WantedState", wantedState);
    Logger.recordOutput("Pivot/setpointDegrees", setpointDegrees);
    Logger.recordOutput("Pivot/CurrentPositionDegrees", getCurrentPositionDegrees());

    // Visualize pivot as pose3d
    // Set the Rotation  of the pivot in the component poses
    Robot.componentPoses[3] =
        new Pose3d(
            PivotConstants.pivotOffsetX,
            PivotConstants.pivotOffsetY,
            Robot.componentPoses[3].getZ(),
            new Rotation3d(0, Units.degreesToRadians(getCurrentPositionDegrees()), 0));
  }

  public SystemState handleStateTransition() {
    switch (wantedState) {
      case IDLE:
        return SystemState.IS_IDLE;
      case VOLTAGE:
        return SystemState.VOLTAGE;
      default:
        // If we are not at the setpoint / goal state, then we are in transition
        return atSetpoint ? SystemState.AT_TARGET : SystemState.IN_TRANSITION;
    }
  }

  /*Exposed Methods for setting and getting state and setpoint */
  public void setWantedStateFunc(WantedState wantedState) {
    this.wantedState = wantedState;
    updateSetpoint(); // Update Immediately
  }

  public Command setWantedState(WantedState wantedState) {
    return Commands.runOnce(() -> setWantedStateFunc(wantedState), this);
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

  public boolean atSetpoint() {
    return MathUtil.isNear(
        setpointDegrees, inputs.pivotPositionDegrees, acceptablePitchErrorDegrees);
  }

  public double getCurrentPositionDegrees() {
    return inputs.pivotPositionDegrees;
  }

  public Command setVoltage(double voltage) {
    return Commands.runOnce(
        () -> {
          setWantedStateFunc(WantedState.VOLTAGE);
          pivotIO.setVoltage(voltage);
        },
        this);
  }

  /*
   *
   * Handle Methods
   *
   */

  public void handleIdling() {
    pivotIO.setVoltage(0.0);
  }

  public void handleStow() {
    setSetpointDegrees(PivotConstants.stowDegrees);
    pivotIO.setSetpointDegrees(setpointDegrees);
  }

  public void handleIntake() {
    setSetpointDegrees(PivotConstants.intakeDegrees);
    pivotIO.setSetpointDegrees(setpointDegrees);
  }

  public void handleL1() {
    setSetpointDegrees(PivotConstants.L1Degrees);
    pivotIO.setSetpointDegrees(setpointDegrees);
  }

  public void handleL2() {
    setSetpointDegrees(PivotConstants.L2Degrees);
    pivotIO.setSetpointDegrees(setpointDegrees);
  }

  public void handleL3() {
    setSetpointDegrees(PivotConstants.L3Degrees);
    pivotIO.setSetpointDegrees(setpointDegrees);
  }

  public void handleL4() {
    setSetpointDegrees(PivotConstants.L4Degrees);
    pivotIO.setSetpointDegrees(setpointDegrees);
  }

  public void handleBarge() {
    setSetpointDegrees(PivotConstants.bargeDegrees);
    pivotIO.setSetpointDegrees(setpointDegrees);
  }

  public void handleProcessor() {
    setSetpointDegrees(PivotConstants.processorDegrees);
    pivotIO.setSetpointDegrees(setpointDegrees);
  }

  public void handleLowAlgea() {
    setSetpointDegrees(PivotConstants.lowAlgeaDegrees);
    pivotIO.setSetpointDegrees(setpointDegrees);
  }

  public void handleHighAlgea() {
    setSetpointDegrees(PivotConstants.highAlgeaDegrees);
    pivotIO.setSetpointDegrees(setpointDegrees);
  }

  public void handleClimb() {
    setSetpointDegrees(PivotConstants.climbDegrees);
    pivotIO.setSetpointDegrees(setpointDegrees);
  }

  public void handleAlgeaIntake() {
    setSetpointDegrees(PivotConstants.algeaIntakeDegrees);
    pivotIO.setSetpointDegrees(setpointDegrees);
  }

  public void handleAlgeaOuttake() {
    setSetpointDegrees(PivotConstants.algeaOuttakeDegrees);
    pivotIO.setSetpointDegrees(setpointDegrees);
  }

  public void handleCoralOuttake() {
    setSetpointDegrees(PivotConstants.coralOuttakeDegrees);
    pivotIO.setSetpointDegrees(setpointDegrees);
  }

  private void updateSetpoint() {
    switch (wantedState) {
      case BARGE:
        setSetpointDegrees(PivotConstants.bargeDegrees);
        break;
      case CLIMB:
        setSetpointDegrees(PivotConstants.climbDegrees);
        break;
      case HIGH_ALGEA:
        setSetpointDegrees(PivotConstants.highAlgeaDegrees);
        break;
      case INTAKE:
        setSetpointDegrees(PivotConstants.intakeDegrees);
        break;
      case ALGEA_INTAKE:
        setSetpointDegrees(PivotConstants.algeaIntakeDegrees);
        break;
      case L1:
        setSetpointDegrees(PivotConstants.L1Degrees);
        break;
      case L2:
        setSetpointDegrees(PivotConstants.L2Degrees);
        break;
      case L3:
        setSetpointDegrees(PivotConstants.L3Degrees);
        break;
      case L4:
        setSetpointDegrees(PivotConstants.L4Degrees);
        break;
      case LOW_ALGEA:
        setSetpointDegrees(PivotConstants.lowAlgeaDegrees);
        break;
      case PROCESSOR:
        setSetpointDegrees(PivotConstants.processorDegrees);
        break;
      case STOW:
        setSetpointDegrees(PivotConstants.stowDegrees);
        break;
      case AlGEA_OUTTAKE:
        setSetpointDegrees(PivotConstants.algeaOuttakeDegrees);
        break;
      case CORAL_OUTTAKE:
        setSetpointDegrees(PivotConstants.coralOuttakeDegrees);
        break;
      case VOLTAGE:
      case IDLE:
      default:
        break;
    }
  }

  public double getSetpointDegrees() {
    return setpointDegrees;
  }
}
