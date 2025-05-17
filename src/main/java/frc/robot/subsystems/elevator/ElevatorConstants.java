package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.Constants;
import frc.robot.util.Gains;

public class ElevatorConstants {
  public static final int leftMotorId = 3;
  public static final int rightMotorId = 4;
  public static final String elevatorCanbus = "3045 Canivore";

  // Current Limits
  public static final double supplyCurrentLimit = 20.0;
  public static final double statorCurrentLimit = 80.0;

  // Inverts
  public static final InvertedValue leftMotorInvert = InvertedValue.Clockwise_Positive;
  public static final InvertedValue rightMotorInvert = InvertedValue.Clockwise_Positive;

  // Gear Ratios
  public static final double elevatorGearRatio = (36.0 / 9.0) * (40.0 / 14.0) * (64.0 / 12.0);
  public static final double drumRadius = 0.0254; // meters
  public static final double ROTATIONS_TO_METERS = 2 * Math.PI * drumRadius;

  // Gains and Constraints
  public static Gains gains =
      switch (Constants.currentMode) {
        case SIM -> new Gains(0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case REAL -> new Gains(0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case REPLAY -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      };

  public static double maxVelocityRotPerSec = 1;
  public static double maxAccelerationRotPerSec = 2;

  // Soft Limits
  public static final double forwardSoftLimitMeters = 0.0;
  public static final double reverseSoftLimitMeters = 2.0;

  // Setpoints
  public static final double stowHeight = 0.0;
  public static final double intakeHeight = 0.0;
  public static final double climbHeight = 0.0;
  public static final double L1Height = 0.0;
  public static final double L2Height = 0.0;
  public static final double L3Height = 0.0;
  public static final double L4Height = 0.0;
  public static final double bargeHeight = 0.0;
  public static final double processorHeight = 0.0;
  public static final double lowAlgeaHeight = 0.0;
  public static final double highAlgeaHeight = 0.0;

  //Tolerance
  public static final double elevatorToleranceMeters = 0.02;
}
