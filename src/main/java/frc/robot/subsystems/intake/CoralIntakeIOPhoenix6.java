// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

/** Add your docs here. */
public class CoralIntakeIOPhoenix6 implements CoralIntakeIO {
  protected final TalonFX rollerMotor =
      new TalonFX(IntakeConstants.coralIntakeRollerID, IntakeConstants.coralIntakeCanbus);
  protected final TalonFX pivotMotor =
      new TalonFX(IntakeConstants.coralIntakePivotID, IntakeConstants.coralIntakeCanbus);

  private final TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

  private final PositionVoltage positionVoltageRequest =
      new PositionVoltage(0.0).withEnableFOC(true);
  private final VelocityVoltage velocityVoltageRequest =
      new VelocityVoltage(0.0).withEnableFOC(true);

  private final StatusSignal<Angle> rotorPositionRot;
  private final StatusSignal<AngularVelocity> rotorVelocityRotPerSec;
  private final StatusSignal<Voltage> pivotVoltage;
  private final StatusSignal<Current> pivotCurrent;
  private final StatusSignal<Temperature> pivotTemperature;
  private final StatusSignal<Voltage> rollerVoltage;
  private final StatusSignal<Current> rollerCurrent;
  private final StatusSignal<Temperature> rollerTemperature;
  private final StatusSignal<AngularVelocity> rollerVelocityRPS;

  public CoralIntakeIOPhoenix6() {
    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.coralPivotSupplyCurrentLimit;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.coralPivotStatorCurrentLimit;
    pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.Audio.BeepOnConfig = false;

    rollerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.coralRollerSupplyCurrentLimit;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.coralRollerStatorCurrentLimit;
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.Audio.BeepOnConfig = false;

    pivotConfig.Slot0.kP = IntakeConstants.coralPivotGains.kP;
    pivotConfig.Slot0.kI = IntakeConstants.coralPivotGains.kI;
    pivotConfig.Slot0.kD = IntakeConstants.coralPivotGains.kD;
    pivotConfig.Slot0.kS = IntakeConstants.coralPivotGains.kS;
    pivotConfig.Slot0.kV = IntakeConstants.coralPivotGains.kV;
    pivotConfig.Slot0.kA = IntakeConstants.coralPivotGains.kA;
    pivotConfig.Slot0.kG = IntakeConstants.coralPivotGains.kG;

    rollerConfig.Slot0.kP = IntakeConstants.coralRollerGains.kP;
    rollerConfig.Slot0.kI = IntakeConstants.coralRollerGains.kI;
    rollerConfig.Slot0.kD = IntakeConstants.coralRollerGains.kD;
    rollerConfig.Slot0.kS = IntakeConstants.coralRollerGains.kS;
    rollerConfig.Slot0.kV = IntakeConstants.coralRollerGains.kV;
    rollerConfig.Slot0.kA = IntakeConstants.coralRollerGains.kA;
    rollerConfig.Slot0.kG = IntakeConstants.coralRollerGains.kG;

    pivotMotor.getConfigurator().apply(pivotConfig);
    rollerMotor.getConfigurator().apply(rollerConfig);

    rotorPositionRot = pivotMotor.getPosition();
    rotorVelocityRotPerSec = pivotMotor.getVelocity();
    pivotVoltage = pivotMotor.getMotorVoltage();
    pivotCurrent = pivotMotor.getSupplyCurrent();
    pivotTemperature = pivotMotor.getDeviceTemp();
    rollerVoltage = rollerMotor.getMotorVoltage();
    rollerCurrent = rollerMotor.getSupplyCurrent();
    rollerTemperature = rollerMotor.getDeviceTemp();
    rollerVelocityRPS = rollerMotor.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        rotorPositionRot,
        rotorVelocityRotPerSec,
        pivotVoltage,
        pivotCurrent,
        pivotTemperature,
        rollerVoltage,
        rollerCurrent,
        rollerTemperature,
        rollerVelocityRPS);

    pivotMotor.optimizeBusUtilization();

    PhoenixUtil.registerSignals(
        true,
        rotorPositionRot,
        rotorVelocityRotPerSec,
        pivotVoltage,
        pivotCurrent,
        pivotTemperature,
        rollerVoltage,
        rollerCurrent,
        rollerTemperature,
        rollerVelocityRPS);
  }

  @Override
  public void setPivotGains(
      double kP, double kI, double kD, double kS, double kA, double kV, double kG) {

    IntakeConstants.coralPivotGains.updateGains(kP, kI, kD, kS, kV, kA, kG);
    pivotConfig.Slot0.kP = kP;
    pivotConfig.Slot0.kI = kI;
    pivotConfig.Slot0.kD = kD;
    pivotConfig.Slot0.kS = kS;
    pivotConfig.Slot0.kV = kV;
    pivotConfig.Slot0.kA = kA;
    pivotConfig.Slot0.kG = kG;
    pivotMotor.getConfigurator().apply(pivotConfig);
  }

  @Override
  public void updateInputs(CoralIntakeIOInputs inputs) {
    inputs.pivotMotorConnected =
        BaseStatusSignal.isAllGood(
            rotorPositionRot, rotorVelocityRotPerSec, pivotVoltage, pivotCurrent, pivotTemperature);

    inputs.pivotVoltage = pivotVoltage.getValueAsDouble();
    inputs.pivotCurrent = pivotCurrent.getValueAsDouble();
    inputs.pivotTemperature = pivotTemperature.getValueAsDouble();
    inputs.pivotPositionDegrees = rotorPositionRot.getValueAsDouble() * 360;
    inputs.pivotVelocityDegreesPerSec = rotorVelocityRotPerSec.getValueAsDouble() * 360;

    inputs.rollerMotorConnected =
        BaseStatusSignal.isAllGood(
            rollerVoltage, rollerCurrent, rollerTemperature, rollerVelocityRPS);
    inputs.rollerVoltage = rollerVoltage.getValueAsDouble();
    inputs.rollerCurrent = rollerCurrent.getValueAsDouble();
    inputs.rollerTemperature = rollerTemperature.getValueAsDouble();
    inputs.rollerVelocityRPS = rollerVelocityRPS.getValueAsDouble();
    inputs.hasCoral = hasCoral();
  }

  @Override
  public void setPivotCurrent(double current) {
    pivotMotor.setControl(new TorqueCurrentFOC(current));
  }

  @Override
  public void setPivotVoltage(double voltage) {
    pivotMotor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void setSetpointDegrees(double setpointInDegrees) {
    pivotMotor.setControl(
        positionVoltageRequest.withPosition(Units.degreesToRotations(setpointInDegrees)));
  }

  @Override
  public void setPositionDegrees(double position) {
    pivotMotor.setPosition(Units.degreesToRotations(position));
  }

  @Override
  public void runIntakeForward() {
    rollerMotor.setControl(velocityVoltageRequest.withVelocity(IntakeConstants.coralRollerForwardSpeed));
  }

  @Override
  public void runIntakeReverse() {
    rollerMotor.setControl(velocityVoltageRequest.withVelocity(IntakeConstants.coralRollerReverseSpeed));
  }

  @Override
  public void stopIntake() {
    rollerMotor.stopMotor();
  }

  public boolean hasCoral() {
    // IMPLEMENT CANRANGE LATER
    return false;
  }
}
