package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class PivotIOPhoenix6 implements PivotIO {
    private final TalonFX pivotMotor;
    private static final double PIVOT_GEAR_RATIO = (36.0 / 9.0) * (40.0 / 14.0) * (64.0 / 12.0);
    private static final double MOTOR_ROTATIONS_PER_DEGREE = PIVOT_GEAR_RATIO / 360.0;
    private static final double DEGREES_PER_ROTATION = 1.0 / MOTOR_ROTATIONS_PER_DEGREE;
    private static final int pivotMotorId = 1;
    private static final String pivotCanbus = "3045 Canivore";

    private final TalonFXConfiguration config = new TalonFXConfiguration();
    
    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0)
        .withEnableFOC(true);

    public PivotIOPhoenix6() {
        pivotMotor = new TalonFX(pivotMotorId, pivotCanbus);

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.CurrentLimits.SupplyCurrentLimit = 20.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.Audio.BeepOnConfig = false;

        config.Slot0.kP = 0.3;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kV = 0;
        config.Slot0.kA = 0;
        config.Slot0.kS = 0;

        pivotMotor.getConfigurator().apply(config);
    }

    @Override
    public void setPID(double kP, double kI, double kD){
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        pivotMotor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs){
        inputs.connected = BaseStatusSignal.refreshAll(
                        pivotMotor.getMotorVoltage(),
                        pivotMotor.getSupplyCurrent(),
                        pivotMotor.getDeviceTemp(),
                        pivotMotor.getPosition(),
                        pivotMotor.getVelocity())
                .isOK();

        inputs.pivotVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();
        inputs.pivotCurrent = pivotMotor.getSupplyCurrent().getValueAsDouble();
        inputs.pivotTemperature = pivotMotor.getDeviceTemp().getValueAsDouble();
        inputs.pivotPositionDegrees = pivotMotor.getPosition().getValueAsDouble() * DEGREES_PER_ROTATION;
        inputs.pivotVelocityDegreesPerSec = pivotMotor.getVelocity().getValueAsDouble() * DEGREES_PER_ROTATION;

    }
        
}
