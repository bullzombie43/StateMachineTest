// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
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
public class AlgeaPivotIOPhoenix6 implements AlgeaPivotIO {
  protected final TalonFX pivotMotor =
      new TalonFX(IntakeConstants.algeaIntakePivotID, IntakeConstants.algeaIntakeCanbus);

  private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

  private final PositionVoltage positionVoltageRequest =
      new PositionVoltage(0.0).withEnableFOC(true);

  private final StatusSignal<Angle> rotorPositionRot;
  private final StatusSignal<AngularVelocity> rotorVelocityRotPerSec;
  private final StatusSignal<Voltage> pivotVoltage;
  private final StatusSignal<Current> pivotCurrent;
  private final StatusSignal<Temperature> pivotTemperature;

  public AlgeaPivotIOPhoenix6() {
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
  public void updateInputs(AlgeaPivotIOInputs inputs) {
    inputs.motorConnected =
        BaseStatusSignal.isAllGood(
            rotorPositionRot, rotorVelocityRotPerSec, pivotVoltage, pivotCurrent, pivotTemperature);

    inputs.pivotVoltage = pivotVoltage.getValueAsDouble();
    inputs.pivotCurrent = pivotCurrent.getValueAsDouble();
    inputs.pivotTemperature = pivotTemperature.getValueAsDouble();
    inputs.pivotPositionDegrees = rotorPositionRot.getValueAsDouble() * 360;
    inputs.pivotVelocityDegreesPerSec = rotorVelocityRotPerSec.getValueAsDouble() * 360;
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
}
