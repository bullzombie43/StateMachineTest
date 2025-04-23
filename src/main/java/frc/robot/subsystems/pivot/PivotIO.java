package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    default void setSetpointDegrees(double setpointInDegrees){}

    default void setZeroPosition(double position){}

    default void updateInputs(PivotIOInputs inputs){}

    default void setVoltage(double voltage){}

    default void setCurrent(double current){}

    default void setPID(double kP, double kI, double kD){}

    @AutoLog
    class PivotIOInputs{
        public boolean connected = true;
        public double pivotVoltage = 0.0;
        public double pivotCurrent = 0.0;
        public double pivotTemperature = 0.0;
        public double pivotPositionDegrees = 0.0;
        public double pivotVelocityDegreesPerSec = 0.0;
    }

}
