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
public class AlgeaIntakeIOPhoenix6 implements AlgeaIntakeIO {
  protected final TalonFX pivotMotor =
      new TalonFX(IntakeConstants.algeaIntakePivotID, IntakeConstants.algeaIntakeCanbus);

  protected final TalonFX rollerMotor =
      new TalonFX(IntakeConstants.algeaIntakeRollerID, IntakeConstants.algeaIntakeCanbus);

  private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

  private final PositionVoltage positionVoltageRequest =
      new PositionVoltage(0.0).withEnableFOC(true);

  private final StatusSignal<Angle> rotorPositionRot;
  private final StatusSignal<AngularVelocity> rotorVelocityRotPerSec;
  private final StatusSignal<Voltage> pivotVoltage;
  private final StatusSignal<Current> pivotCurrent;
  private final StatusSignal<Temperature> pivotTemperature;

  private final StatusSignal<Voltage> rollerVoltage;
  private final StatusSignal<Current> rollerCurrent;
  private final StatusSignal<Temperature> rollerTemperature;
  private final StatusSignal<AngularVelocity> rollerVelocityRPS;

  public AlgeaIntakeIOPhoenix6() {
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    motorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.algeaPivotSupplyCurrentLimit;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.algeaPivotStatorCurrentLimit;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.Audio.BeepOnConfig = false;

    motorConfig.Slot0.kP = IntakeConstants.algeaPivotGains.kP;
    motorConfig.Slot0.kI = IntakeConstants.algeaPivotGains.kI;
    motorConfig.Slot0.kD = IntakeConstants.algeaPivotGains.kD;
    motorConfig.Slot0.kS = IntakeConstants.algeaPivotGains.kS;
    motorConfig.Slot0.kV = IntakeConstants.algeaPivotGains.kV;
    motorConfig.Slot0.kA = IntakeConstants.algeaPivotGains.kA;
    motorConfig.Slot0.kG = IntakeConstants.algeaPivotGains.kG;

    pivotMotor.getConfigurator().apply(motorConfig);

    rotorPositionRot = pivotMotor.getPosition();
    rotorVelocityRotPerSec = pivotMotor.getVelocity();
    pivotVoltage = pivotMotor.getMotorVoltage();
    pivotCurrent = pivotMotor.getSupplyCurrent();
    pivotTemperature = pivotMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        rotorPositionRot,
        rotorVelocityRotPerSec,
        pivotVoltage,
        pivotCurrent,
        pivotTemperature);

    pivotMotor.optimizeBusUtilization();

    PhoenixUtil.registerSignals(
        true,
        rotorPositionRot,
        rotorVelocityRotPerSec,
        pivotVoltage,
        pivotCurrent,
        pivotTemperature);

    rollerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.algeaRollerSupplyCurrentLimit;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.algeaRollerStatorCurrentLimit;
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    rollerConfig.Slot0.kP = IntakeConstants.algeaRollerGains.kP;
    rollerConfig.Slot0.kI = IntakeConstants.algeaRollerGains.kI;
    rollerConfig.Slot0.kD = IntakeConstants.algeaRollerGains.kD;
    rollerConfig.Slot0.kS = IntakeConstants.algeaRollerGains.kS;
    rollerConfig.Slot0.kV = IntakeConstants.algeaRollerGains.kV;
    rollerConfig.Slot0.kA = IntakeConstants.algeaRollerGains.kA;
    rollerConfig.Slot0.kG = IntakeConstants.algeaRollerGains.kG;

    rollerMotor.getConfigurator().apply(rollerConfig);

    rollerVoltage = rollerMotor.getMotorVoltage();
    rollerCurrent = rollerMotor.getSupplyCurrent();
    rollerTemperature = rollerMotor.getDeviceTemp();
    rollerVelocityRPS = rollerMotor.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, rollerVoltage, rollerCurrent, rollerTemperature, rollerVelocityRPS);

    PhoenixUtil.registerSignals(
        true, rollerVoltage, rollerCurrent, rollerTemperature, rollerVelocityRPS);
  }

  @Override
  public void setGains(
      double kP, double kI, double kD, double kS, double kA, double kV, double kG) {

    IntakeConstants.algeaPivotGains.updateGains(kP, kI, kD, kS, kV, kA, kG);
    motorConfig.Slot0.kP = kP;
    motorConfig.Slot0.kI = kI;
    motorConfig.Slot0.kD = kD;
    motorConfig.Slot0.kS = kS;
    motorConfig.Slot0.kV = kV;
    motorConfig.Slot0.kA = kA;
    motorConfig.Slot0.kG = kG;
    pivotMotor.getConfigurator().apply(motorConfig);
  }

  @Override
  public void updateInputs(AlgeaIntakeIOInputs inputs) {
    inputs.motorConnected =
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
    inputs.hasAlgea = hasAlgea();
  }

  @Override
  public void setCurrent(double current) {
    pivotMotor.setControl(new TorqueCurrentFOC(current));
  }

  @Override
  public void setVoltage(double voltage) {
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
  public void runRollerForward() {
    rollerMotor.setControl(
        new VelocityVoltage(IntakeConstants.algeaRollerForwardSpeed).withEnableFOC(true));
  }

  @Override
  public void runRollerReverse() {
    rollerMotor.setControl(
        new VelocityVoltage(IntakeConstants.algeaRollerReverseSpeed).withEnableFOC(true));
  }

  @Override
  public void stopRoller() {
    rollerMotor.stopMotor();
  }

  public boolean hasAlgea() {
    return false; // Implement Later
  }
}
