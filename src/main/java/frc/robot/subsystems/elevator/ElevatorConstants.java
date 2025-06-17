package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.Gains;

public class ElevatorConstants {
  public static final int leftMotorId = 3;
  public static final int rightMotorId = 4;
  public static final String elevatorCanbus = "3045 Canivore";

  // Current Limits
  public static final double supplyCurrentLimit = 40.0;
  public static final double statorCurrentLimit = 80.0;

  // Inverts
  public static final InvertedValue leftMotorInvert = InvertedValue.Clockwise_Positive;
  public static final InvertedValue rightMotorInvert = InvertedValue.Clockwise_Positive;

  // Gear Ratios
  public static final double elevatorGearRatio = (56.0 / 12.0);
  public static final double drumRadius = 0.0286; // meters
  public static final double ROTATIONS_TO_METERS = 2 * Math.PI * drumRadius;

  // Gains and Constraints
  public static Gains gains =
      switch (Constants.currentMode) {
        case SIM -> new Gains(10, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0);
        case REAL -> new Gains(0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case REPLAY -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      };

  public static double maxVelocityRotPerSec =
      Constants.currentMode == Constants.simMode ? 1000 : 22;
  public static double maxAccelerationRotPerSec =
      Constants.currentMode == Constants.simMode ? 1000 : 22;

  // Soft Limits
  public static final double forwardSoftLimitMeters = 0.0;
  public static final double reverseSoftLimitMeters = 2.0;

  // Setpoints
  public static final double stowHeight = 0.0;
  public static final double intakeHeight = 0.0;
  public static final double climbHeight = 0.0;
  public static final double L1Height = 0.0;
  public static final double L2Height = 0.2;
  public static final double L3Height = 0.6;
  public static final double L4Height = 1.45;
  public static final double bargeHeight = 0.0;
  public static final double processorHeight = 0.0;
  public static final double lowAlgeaHeight = 0.0;
  public static final double highAlgeaHeight = 0.0;

  // Tolerance
  public static final double elevatorToleranceMeters = 0.02;

  // Simulation Values
  public static final double carriageMassKg = 10;
  public static final double stage1XOffset = Units.inchesToMeters(-6.8);
  public static final double stage1YOffset = Units.inchesToMeters(5.8);
  public static final double stage1ZOffset = Units.inchesToMeters(1.7);
  public static final double stage1Length = Units.inchesToMeters(31.642);
}
