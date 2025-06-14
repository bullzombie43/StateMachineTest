package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class ElevatorIOPhoenix6 implements ElevatorIO {
  protected final TalonFX leftMotor = new TalonFX(leftMotorId, elevatorCanbus);
  protected final TalonFX rightMotor = new TalonFX(rightMotorId, elevatorCanbus);

  private final TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();

  @SuppressWarnings("unused")
  private final PositionVoltage positionVoltageRequest =
      new PositionVoltage(0.0).withEnableFOC(true);

  private final MotionMagicVoltage motionMagicVoltageRequest =
      new MotionMagicVoltage(0.0).withEnableFOC(true);

  private final StatusSignal<Angle> leftRotorPositionRot;
  private final StatusSignal<AngularVelocity> leftRotorVelocityRotPerSec;
  private final StatusSignal<Voltage> leftMotorVoltage;
  private final StatusSignal<Current> leftMotorCurrent;
  private final StatusSignal<Temperature> leftMotorTemperature;
  private final StatusSignal<Angle> rightRotorPositionRot;
  private final StatusSignal<AngularVelocity> rightRotorVelocityRotPerSec;
  private final StatusSignal<Voltage> rightMotorVoltage;
  private final StatusSignal<Current> rightMotorCurrent;
  private final StatusSignal<Temperature> rightMotorTemperature;
  private final StatusSignal<Angle> elevatorPositionRot;
  private final StatusSignal<AngularVelocity> elevatorVelocityRotPerSec;

  public ElevatorIOPhoenix6() {
    rightMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    rightMotorConfig.Feedback.SensorToMechanismRatio = elevatorGearRatio;
    rightMotorConfig.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    rightMotorConfig.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
    rightMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rightMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightMotorConfig.MotorOutput.Inverted = rightMotorInvert;
    rightMotorConfig.Audio.BeepOnConfig = false;

    rightMotorConfig.Slot0.kP = gains.kP;
    rightMotorConfig.Slot0.kI = gains.kI;
    rightMotorConfig.Slot0.kD = gains.kD;
    rightMotorConfig.Slot0.kA = gains.kA;
    rightMotorConfig.Slot0.kS = gains.kS;
    rightMotorConfig.Slot0.kV = gains.kV;
    rightMotorConfig.Slot0.kG = gains.kG;
    rightMotorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    rightMotorConfig.MotionMagic.MotionMagicAcceleration = maxAccelerationRotPerSec;
    rightMotorConfig.MotionMagic.MotionMagicCruiseVelocity = maxVelocityRotPerSec;

    PhoenixUtil.copyConfig(leftMotorConfig, rightMotorConfig);
    leftMotorConfig.MotorOutput.Inverted = leftMotorInvert;

    // Adjust Inverts and Apply config
    leftMotor.getConfigurator().apply(rightMotorConfig);
    rightMotor.getConfigurator().apply(leftMotorConfig);

    // Status Signals
    leftRotorPositionRot = leftMotor.getRotorPosition();
    leftRotorVelocityRotPerSec = leftMotor.getRotorVelocity();
    leftMotorVoltage = leftMotor.getMotorVoltage();
    leftMotorCurrent = leftMotor.getStatorCurrent();
    leftMotorTemperature = leftMotor.getDeviceTemp();
    rightRotorPositionRot = rightMotor.getRotorPosition();
    rightRotorVelocityRotPerSec = rightMotor.getRotorVelocity();
    rightMotorVoltage = rightMotor.getMotorVoltage();
    rightMotorCurrent = rightMotor.getStatorCurrent();
    rightMotorTemperature = rightMotor.getDeviceTemp();

    // Use right Motor for elevator position and velocity
    elevatorPositionRot = rightMotor.getPosition();
    elevatorVelocityRotPerSec = rightMotor.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        leftRotorPositionRot,
        leftRotorVelocityRotPerSec,
        leftMotorVoltage,
        leftMotorCurrent,
        leftMotorTemperature,
        rightRotorPositionRot,
        rightRotorVelocityRotPerSec,
        rightMotorVoltage,
        rightMotorCurrent,
        rightMotorTemperature,
        elevatorPositionRot,
        elevatorVelocityRotPerSec);

    leftMotor.optimizeBusUtilization();
    rightMotor.optimizeBusUtilization();

    PhoenixUtil.registerSignals(
        true,
        leftRotorPositionRot,
        leftRotorVelocityRotPerSec,
        leftMotorVoltage,
        leftMotorCurrent,
        leftMotorTemperature,
        rightRotorPositionRot,
        rightRotorVelocityRotPerSec,
        rightMotorVoltage,
        rightMotorCurrent,
        rightMotorTemperature,
        elevatorPositionRot,
        elevatorVelocityRotPerSec);
  }

  @Override
  public void setGains(
      double kP, double kI, double kD, double kS, double kA, double kV, double kG) {
    gains.updateGains(kP, kI, kD, kS, kV, kA, kG);
    rightMotorConfig.Slot0.kP = gains.kP;
    rightMotorConfig.Slot0.kI = gains.kI;
    rightMotorConfig.Slot0.kD = gains.kD;
    rightMotorConfig.Slot0.kA = gains.kA;
    rightMotorConfig.Slot0.kS = gains.kS;
    rightMotorConfig.Slot0.kV = gains.kV;
    rightMotorConfig.Slot0.kG = gains.kG;
    leftMotorConfig.Slot0 = rightMotorConfig.Slot0;
    rightMotor.getConfigurator().apply(rightMotorConfig);
    leftMotor.getConfigurator().apply(leftMotorConfig);
  }

  @Override
  public void setProfileConstraints(double maxVelocity, double maxAcceleration) {
    ElevatorConstants.maxAccelerationRotPerSec = maxAcceleration;
    ElevatorConstants.maxVelocityRotPerSec = maxVelocity;
    rightMotorConfig.MotionMagic.MotionMagicAcceleration =
        ElevatorConstants.maxAccelerationRotPerSec;
    rightMotorConfig.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.maxVelocityRotPerSec;
    leftMotorConfig.MotionMagic = rightMotorConfig.MotionMagic;
    rightMotor.getConfigurator().apply(rightMotorConfig);
    leftMotor.getConfigurator().apply(leftMotorConfig);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leftMotorConnected = leftMotor.isConnected();
    inputs.rightMotorConnected = rightMotor.isConnected();
    inputs.leftVoltage = leftMotorVoltage.getValueAsDouble();
    inputs.rightVoltage = rightMotorVoltage.getValueAsDouble();
    inputs.leftCurrent = leftMotorCurrent.getValueAsDouble();
    inputs.rightCurrent = rightMotorCurrent.getValueAsDouble();
    inputs.leftTemperature = leftMotorTemperature.getValueAsDouble();

    inputs.elevatorHeightMeters =
        Elevator.rotationsToMeters(elevatorPositionRot.getValueAsDouble());
    inputs.elevatorVelocityMetersPerSec =
        Elevator.rotationsToMeters(elevatorVelocityRotPerSec.getValueAsDouble());
  }

  @Override
  public void setCurrent(double leftCurrent, double rightCurrent) {
    leftMotor.setControl(new TorqueCurrentFOC(leftCurrent));
    rightMotor.setControl(new TorqueCurrentFOC(rightCurrent));
  }

  @Override
  public void setVoltage(double leftVoltage, double rightVoltage) {
    leftMotor.setControl(new VoltageOut(leftVoltage));
    rightMotor.setControl(new VoltageOut(rightVoltage));
  }

  @Override
  public void setSetpointMeters(double setpointInMeters) {
    double setpointRotations = Elevator.metersToRotations(setpointInMeters);
    leftMotor.setControl(motionMagicVoltageRequest.withPosition(setpointRotations));
    rightMotor.setControl(motionMagicVoltageRequest.withPosition(setpointRotations));
  }

  @Override
  public void setPositionMeters(double positionMeters) {
    leftMotor.setPosition(Elevator.metersToRotations(positionMeters));
    rightMotor.setPosition(Elevator.metersToRotations(positionMeters));
  }
}
