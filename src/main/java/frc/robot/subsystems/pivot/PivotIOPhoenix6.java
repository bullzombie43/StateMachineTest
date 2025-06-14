package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class PivotIOPhoenix6 implements PivotIO {
  protected final TalonFX pivotMotor = new TalonFX(pivotMotorId, pivotCanbus);
  protected final CANcoder pivotEncoder = new CANcoder(pivotEncoderId, pivotCanbus);

  private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
  private final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

  @SuppressWarnings("unused")
  private final PositionVoltage positionVoltageRequest =
      new PositionVoltage(0.0).withEnableFOC(true);

  private final MotionMagicVoltage motionMagicVoltageRequest =
      new MotionMagicVoltage(0.0).withEnableFOC(true);

  private final StatusSignal<Angle> rotorPositionRot;
  private final StatusSignal<AngularVelocity> rotorVelocityRotPerSec;
  private final StatusSignal<Voltage> pivotVoltage;
  private final StatusSignal<Current> pivotCurrent;
  private final StatusSignal<Temperature> pivotTemperature;
  private final StatusSignal<Angle> absolutePositionRot;
  private final StatusSignal<AngularVelocity> absoluteVelocityRotPerSec;

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

    // Status Signals
    rotorPositionRot = pivotMotor.getPosition();
    rotorVelocityRotPerSec = pivotMotor.getVelocity();
    pivotVoltage = pivotMotor.getMotorVoltage();
    pivotCurrent = pivotMotor.getSupplyCurrent();
    pivotTemperature = pivotMotor.getDeviceTemp();
    absolutePositionRot = pivotEncoder.getAbsolutePosition();
    absoluteVelocityRotPerSec = pivotEncoder.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        rotorPositionRot,
        rotorVelocityRotPerSec,
        pivotVoltage,
        pivotCurrent,
        pivotTemperature);

    BaseStatusSignal.setUpdateFrequencyForAll(200, absolutePositionRot, absoluteVelocityRotPerSec);

    pivotMotor.optimizeBusUtilization();
    pivotEncoder.optimizeBusUtilization();

    PhoenixUtil.registerSignals(
        true,
        rotorPositionRot,
        rotorVelocityRotPerSec,
        pivotVoltage,
        pivotCurrent,
        pivotTemperature,
        absolutePositionRot,
        absoluteVelocityRotPerSec);
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
        BaseStatusSignal.isAllGood(
            rotorPositionRot, rotorVelocityRotPerSec, pivotVoltage, pivotCurrent, pivotTemperature);

    inputs.encoderConnected =
        BaseStatusSignal.isAllGood(absolutePositionRot, absoluteVelocityRotPerSec);

    inputs.pivotVoltage = pivotVoltage.getValueAsDouble();
    inputs.pivotCurrent = pivotCurrent.getValueAsDouble();
    inputs.pivotTemperature = pivotTemperature.getValueAsDouble();
    inputs.pivotPositionDegrees = rotorPositionRot.getValueAsDouble() * DEGREES_PER_MOTOR_ROTATION;
    inputs.pivotVelocityDegreesPerSec =
        rotorVelocityRotPerSec.getValueAsDouble() * DEGREES_PER_MOTOR_ROTATION;

    // The encoder is direct to the pivot shaft so just multiply by 360 to convert from rotation to
    // degrees
    inputs.absolutePositionDegrees = absolutePositionRot.getValueAsDouble() * 360.0;
    inputs.absoluteVelocityDegreesPerSec = absoluteVelocityRotPerSec.getValueAsDouble() * 360.0;
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
        motionMagicVoltageRequest.withPosition(degreesToMotorRotations(setpointInDegrees)));
  }

  @Override
  public void setPositionDegrees(double position) {
    pivotMotor.setPosition(degreesToMotorRotations(position));
  }

  private double degreesToMotorRotations(double degrees) {
    return degrees * MOTOR_ROTATIONS_PER_DEGREE;
  }
}
