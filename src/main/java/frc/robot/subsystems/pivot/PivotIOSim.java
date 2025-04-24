package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotConstants.PIVOT_GEAR_RATIO;
import static frc.robot.subsystems.pivot.PivotConstants.forwardSoftLimitDegrees;
import static frc.robot.subsystems.pivot.PivotConstants.reverseSoftLimitDegrees;
import static frc.robot.subsystems.pivot.PivotConstants.simArmLength;
import static frc.robot.subsystems.pivot.PivotConstants.simMOI;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotIOSim extends PivotIOPhoenix6{
    private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(
        DCMotor.getKrakenX60Foc(1), 
        PIVOT_GEAR_RATIO, 
        simMOI, 
        simArmLength, 
        reverseSoftLimitDegrees, 
        forwardSoftLimitDegrees, 
        false, 
        0.0);

    private Notifier simNotifier = null;

    private double lastUpdateTimestamp;

    public PivotIOSim(){
        simNotifier = new Notifier(() -> {
            updateSimState();
        });

        simNotifier.startPeriodic(0.005); // Run Simulation at a faster rate so PID gains perform more reasonably
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        super.updateInputs(inputs);
    }

    public void updateSimState(){
        var motorSimState = pivotMotor.getSimState();
        var encoderSimState = pivotEncoder.getSimState();

        motorSimState.setSupplyVoltage(12.0);
        encoderSimState.setSupplyVoltage(12.0);

        double simVoltage = motorSimState.getMotorVoltage();
        pivotSim.setInputVoltage(simVoltage);
        Logger.recordOutput("Pivot/Sim/SimVoltage", simVoltage);

        double timestamp = Timer.getTimestamp();
        pivotSim.update(timestamp - lastUpdateTimestamp);
        lastUpdateTimestamp = timestamp;

        double simPositionRadians = pivotSim.getAngleRads();
        double simVelocityRadiansPerSec = pivotSim.getVelocityRadPerSec();
        Logger.recordOutput("Pivot/Sim/SimPositionRads", simPositionRadians);
        Logger.recordOutput("Pivot/Sim/SimVelocityRadiansPerSec", simVelocityRadiansPerSec);
        
        //Mutate Encoder Position and Velocity
        //Encoder is 1:1 with mechanism so we directly set its position to the simulated mechanism position
        encoderSimState.setRawPosition(Units.radiansToRotations(simPositionRadians));
        encoderSimState.setVelocity(Units.radiansToRotations(simVelocityRadiansPerSec));
        Logger.recordOutput("Pivot/Sim/Cancoder Position", pivotEncoder.getPosition().getValueAsDouble());

        //Mutate Motor Position and Velocity
        double rotorPosition = Units.radiansToRotations(simPositionRadians) / PIVOT_GEAR_RATIO;
        motorSimState.setRawRotorPosition(rotorPosition);
        Logger.recordOutput("Pivot/Sim/setRawRotorPosition", rotorPosition);

        double rotorVelocity = Units.radiansToRotations(simVelocityRadiansPerSec) / PIVOT_GEAR_RATIO;
        motorSimState.setRotorVelocity(rotorVelocity);
        Logger.recordOutput("Pivot/Sim/setRotorVelocity", rotorVelocity);
    }

}
