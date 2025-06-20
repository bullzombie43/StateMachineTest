package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.Gains;
import frc.robot.util.LoggedTunableNumber;

public class ElevatorConstants {
  public static final int leftMotorId = 11;
  public static final int rightMotorId = 12;
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
  public static final double reverseSoftLimitMeters = 1.75;

  // Setpoints
  public static double stowHeight = 0.0;
  public static double intakeHeight = 0.0;
  public static double outtakeHeight = 0.0;
  public static double climbHeight = 0.0;
  public static double L1Height = 0.0;
  public static double L2Height = 0.2;
  public static double L3Height = 0.6;
  public static double L4Height = 1.45;
  public static double prepareBargeHeight = 0.7;
  public static double bargeHeight = 1.75;
  public static double processorHeight = 0.0;
  public static double lowAlgeaHeight = 0.0;
  public static double highAlgeaHeight = 0.0;
  public static double algeaIntakeHeight = 0.0;
  public static double algeaOuttakeHeight = 0.0;

  // Tolerance
  public static final double elevatorToleranceMeters = 0.02;

  // Simulation Values
  public static final double carriageMassKg = 10;
  public static final double stage1XOffset = Units.inchesToMeters(-6.8);
  public static final double stage1YOffset = Units.inchesToMeters(5.8);
  public static final double stage1ZOffset = Units.inchesToMeters(1.7);
  public static final double stage1Length = Units.inchesToMeters(31.642);

  // Tunable Numbers
  public static final LoggedTunableNumber l2HeightTunable =
      new LoggedTunableNumber("Elevator/L2Height", L2Height);
  public static final LoggedTunableNumber l3HeightTunable =
      new LoggedTunableNumber("Elevator/L3Height", L3Height);
  public static final LoggedTunableNumber l4HeightTunable =
      new LoggedTunableNumber("Elevator/L4Height", L4Height);
  public static final LoggedTunableNumber prepareBargeHeightTunable =
      new LoggedTunableNumber("Elevator/PrepareBargeHeight", prepareBargeHeight);
  public static final LoggedTunableNumber bargeHeightTunable =
      new LoggedTunableNumber("Elevator/BargeHeight", bargeHeight);
  public static final LoggedTunableNumber processorHeightTunable =
      new LoggedTunableNumber("Elevator/ProcessorHeight", processorHeight);

  public static void checkTunableNumbers() {
    if (l2HeightTunable.hasChanged(ElevatorConstants.class.hashCode())
        || l3HeightTunable.hasChanged(ElevatorConstants.class.hashCode())
        || l4HeightTunable.hasChanged(ElevatorConstants.class.hashCode())
        || prepareBargeHeightTunable.hasChanged(ElevatorConstants.class.hashCode())
        || bargeHeightTunable.hasChanged(ElevatorConstants.class.hashCode())
        || processorHeightTunable.hasChanged(ElevatorConstants.class.hashCode())) {
      L2Height = l2HeightTunable.get();
      L3Height = l3HeightTunable.get();
      L4Height = l4HeightTunable.get();
      prepareBargeHeight = prepareBargeHeightTunable.get();
      bargeHeight = bargeHeightTunable.get();
      processorHeight = processorHeightTunable.get();
    }
  }
}
