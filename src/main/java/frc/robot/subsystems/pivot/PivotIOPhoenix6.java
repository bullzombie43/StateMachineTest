package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class PivotIOPhoenix6 implements PivotIO {
  protected final TalonFX pivotMotor = new TalonFX(pivotMotorId, pivotCanbus);
  protected final CANcoder pivotEncoder = new CANcoder(pivotEncoderId, pivotCanbus);

  private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
  private final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

  private final PositionVoltage positionVoltageRequest =
      new PositionVoltage(0.0).withEnableFOC(true);

  public PivotIOPhoenix6() {
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    motorConfig.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.Audio.BeepOnConfig = false;

    motorConfig.Slot0.kP = gains.kP;
    motorConfig.Slot0.kI = gains.kI;
    motorConfig.Slot0.kD = gains.kD;
    motorConfig.Slot0.kA = gains.kA;
    motorConfig.Slot0.kS = gains.kS;
    motorConfig.Slot0.kV = gains.kV;
    motorConfig.Slot0.kG = gains.kG;
    motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    motorConfig.MotionMagic.MotionMagicAcceleration = maxAccelerationRotPerSec;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = maxVelocityRotPerSec;

    pivotMotor.getConfigurator().apply(motorConfig); // Apply Motor Configuration

    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // [-0.5, 0.5)
    encoderConfig.MagnetSensor.MagnetOffset = magnetOffset;
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    pivotEncoder.getConfigurator().apply(encoderConfig); // Apply Encoder Configuration
  }

  @Override
  public void setGains(
      double kP, double kI, double kD, double kS, double kA, double kV, double kG) {
    gains.updateGains(kP, kI, kD, kS, kV, kA, kG);
    motorConfig.Slot0.kP = gains.kP;
    motorConfig.Slot0.kI = gains.kI;
    motorConfig.Slot0.kD = gains.kD;
    motorConfig.Slot0.kA = gains.kA;
    motorConfig.Slot0.kS = gains.kS;
    motorConfig.Slot0.kV = gains.kV;
    motorConfig.Slot0.kG = gains.kG;
    pivotMotor.getConfigurator().apply(motorConfig);
  }

  @Override
  public void setProfileConstraints(double maxVelocity, double maxAcceleration) {
    maxAccelerationRotPerSec = maxAcceleration;
    maxVelocityRotPerSec = maxVelocity;
    motorConfig.MotionMagic.MotionMagicAcceleration = maxAccelerationRotPerSec;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = maxVelocityRotPerSec;
    pivotMotor.getConfigurator().apply(motorConfig);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.motorConnected =
        BaseStatusSignal.refreshAll(
                pivotMotor.getMotorVoltage(),
                pivotMotor.getSupplyCurrent(),
                pivotMotor.getDeviceTemp(),
                pivotMotor.getPosition(),
                pivotMotor.getVelocity())
            .isOK();

    inputs.encoderConnected =
        BaseStatusSignal.refreshAll(pivotEncoder.getAbsolutePosition(), pivotEncoder.getVelocity())
            .isOK();

    inputs.pivotVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();
    inputs.pivotCurrent = pivotMotor.getSupplyCurrent().getValueAsDouble();
    inputs.pivotTemperature = pivotMotor.getDeviceTemp().getValueAsDouble();
    inputs.pivotPositionDegrees =
        pivotMotor.getPosition().getValueAsDouble() * DEGREES_PER_ROTATION;
    inputs.pivotVelocityDegreesPerSec =
        pivotMotor.getVelocity().getValueAsDouble() * DEGREES_PER_ROTATION;

    // The encoder is direct to the pivot shaft so just multiply by 360 to convert from rotation to
    // degrees
    inputs.absolutePositionDegrees = pivotEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
    inputs.absoluteVelocityDegreesPerSec = pivotEncoder.getVelocity().getValueAsDouble() * 360.0;
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
        positionVoltageRequest.withPosition(degreesToMotorRotations(setpointInDegrees)));
  }

  @Override
  public void setZeroPosition(double position) {
    pivotMotor.setPosition(degreesToMotorRotations(position));
  }

  private double degreesToMotorRotations(double degrees) {
    return degrees * MOTOR_ROTATIONS_PER_DEGREE;
  }
}
