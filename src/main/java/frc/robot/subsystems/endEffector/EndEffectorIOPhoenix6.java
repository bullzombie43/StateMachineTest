// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

/** Add your docs here. */
public class EndEffectorIOPhoenix6 implements EndEffectorIO {
  protected final TalonFX motor =
      new TalonFX(EndEffectorConstants.endEffectorMotorID, EndEffectorConstants.endEffectorCanbus);

  private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

  private final VelocityVoltage velocityVoltageRequest =
      new VelocityVoltage(0.0).withEnableFOC(true);

  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Current> motorCurrent;
  private final StatusSignal<Temperature> motorTemperature;
  private final StatusSignal<AngularVelocity> motorVelocityRPS;

  public EndEffectorIOPhoenix6() {
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    motorConfig.CurrentLimits.SupplyCurrentLimit =
        EndEffectorConstants.endEffectorSupplyCurrentLimit;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit =
        EndEffectorConstants.endEffectorStatorCurrentLimit;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.Audio.BeepOnConfig = false;

    motorConfig.Slot0.kP = EndEffectorConstants.gains.kP;
    motorConfig.Slot0.kI = EndEffectorConstants.gains.kI;
    motorConfig.Slot0.kD = EndEffectorConstants.gains.kD;
    motorConfig.Slot0.kS = EndEffectorConstants.gains.kS;
    motorConfig.Slot0.kV = EndEffectorConstants.gains.kV;
    motorConfig.Slot0.kA = EndEffectorConstants.gains.kA;
    motorConfig.Slot0.kG = EndEffectorConstants.gains.kG;

    motor.getConfigurator().apply(motorConfig);

    motorVoltage = motor.getMotorVoltage();
    motorCurrent = motor.getStatorCurrent();
    motorTemperature = motor.getDeviceTemp();
    motorVelocityRPS = motor.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, motorVoltage, motorCurrent, motorTemperature, motorVelocityRPS);

    motor.optimizeBusUtilization();

    PhoenixUtil.registerSignals(
        true, motorVoltage, motorCurrent, motorTemperature, motorVelocityRPS);
  }

  @Override
  public void setGains(
      double kP, double kI, double kD, double kS, double kA, double kV, double kG) {
    motorConfig.Slot0.kP = kP;
    motorConfig.Slot0.kI = kI;
    motorConfig.Slot0.kD = kD;
    motorConfig.Slot0.kS = kS;
    motorConfig.Slot0.kV = kV;
    motorConfig.Slot0.kA = kA;
    motorConfig.Slot0.kG = kG;

    motor.getConfigurator().apply(motorConfig);
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    inputs.endEffectorMotorConnected = motor.isConnected();
    inputs.endEffectorVoltage = motorVoltage.getValueAsDouble();
    inputs.endEffectorCurrent = motorCurrent.getValueAsDouble();
    inputs.endEffectorTemperature = motorTemperature.getValueAsDouble();
    inputs.endEffectorVelocityRPS = motorVelocityRPS.getValueAsDouble();
    inputs.hasCoral = hasCoral();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void setCurrent(double current) {
    motor.setControl(new TorqueCurrentFOC(current));
  }

  @Override
  public void runEndEffectorForward() {
    motor.setControl(velocityVoltageRequest.withVelocity(EndEffectorConstants.forwardSpeed));
  }

  @Override
  public void runEndEffectorReverse() {
    motor.setControl(velocityVoltageRequest.withVelocity(EndEffectorConstants.reverseSpeed));
  }

  @Override
  public void stopEndEffector() {
    motor.stopMotor();
  }

  public boolean hasCoral() {
    // Implement with canRange later
    return false;
  }
}
