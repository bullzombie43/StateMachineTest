// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.Gains;
import frc.robot.util.LoggedTunableNumber;

/** Add your docs here. */
public class PivotConstants {
  public static final int pivotMotorId = 9;
  public static final int pivotEncoderId = 10;
  public static final String pivotCanbus = "3045 Canivore";

  public static final double PIVOT_GEAR_RATIO = (36.0 / 9.0) * (40.0 / 14.0) * (64.0 / 12.0);
  public static final double MOTOR_ROTATIONS_PER_DEGREE = PIVOT_GEAR_RATIO / 360.0;
  public static final double DEGREES_PER_MOTOR_ROTATION = 1.0 / MOTOR_ROTATIONS_PER_DEGREE;

  public static final double supplyCurrentLimit = 40.0;
  public static final double statorCurrentLimit = 80.0;

  public static final double magnetOffset = Constants.currentMode == Constants.simMode ? 0.0 : 0.0;

  public static Gains gains =
      switch (Constants.currentMode) {
        case SIM -> new Gains(1.0, 0.0, 0, 0.0, 0.0, 0.0, 0.0);
        case REAL -> new Gains(0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case REPLAY -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      };
  public static double maxVelocityRotPerSec =
      Constants.currentMode == Constants.simMode ? 1000 : 10;
  public static double maxAccelerationRotPerSec =
      Constants.currentMode == Constants.simMode ? 1000 : 10;

  public static final double forwardSoftLimitDegrees = 180.0;
  public static final double reverseSoftLimitDegrees = -180.0;

  public static final double acceptablePitchErrorDegrees = 1.0;

  /* Setpoints */
  public static double stowDegrees = 0.0;
  public static double intakeDegrees = -180.0;
  public static double climbDegrees = 0.0;
  public static double L1Degrees = 0;
  public static double L2Degrees = -30.0;
  public static double L3Degrees = -30.0;
  public static double L4Degrees = -65.0;
  public static double prepareBargeDegrees = -45.0;
  public static double bargeDegrees = -45;
  public static double processorDegrees = -180.0;
  public static double lowAlgeaDegrees = -180.0;
  public static double highAlgeaDegrees = -180.0;
  public static double algeaIntakeDegrees = -180.0;
  public static double algeaOuttakeDegrees = -180.0;
  public static double coralOuttakeDegrees = -180.0;

  /* Simulation Values */
  public static final double simMOI = 0.5; // kg*m^2
  public static final double simArmLength = 0.15; // meters
  public static final double pivotOffsetX = Units.inchesToMeters(-6.8);
  public static final double pivotOffsetY = 0;
  public static final double pivotOffsetZ = Units.inchesToMeters(20.481);

  // Tunable Numbers
  public static final LoggedTunableNumber l2HeightTunable =
      new LoggedTunableNumber("Pivot/L2Degrees", L2Degrees);
  public static final LoggedTunableNumber l3HeightTunable =
      new LoggedTunableNumber("Pivot/L3Degrees", L3Degrees);
  public static final LoggedTunableNumber l4HeightTunable =
      new LoggedTunableNumber("Pivot/L4Degrees", L4Degrees);
  public static final LoggedTunableNumber prepareBargeHeightTunable =
      new LoggedTunableNumber("Pivot/PrepareBargeDegrees", prepareBargeDegrees);
  public static final LoggedTunableNumber bargeHeightTunable =
      new LoggedTunableNumber("Pivot/BargeDegrees", bargeDegrees);
  public static final LoggedTunableNumber processorHeightTunable =
      new LoggedTunableNumber("Pivot/ProcessorDegrees", processorDegrees);
  public static final LoggedTunableNumber lowAlgeaHeightTunable =
      new LoggedTunableNumber("Pivot/LowAlgeaDegrees", lowAlgeaDegrees);
  public static final LoggedTunableNumber highAlgeaHeightTunable =
      new LoggedTunableNumber("Pivot/HighAlgeaDegrees", highAlgeaDegrees);

  public static void checkTunableNumbers() {
    if (l2HeightTunable.hasChanged(PivotConstants.class.hashCode())
        || l3HeightTunable.hasChanged(PivotConstants.class.hashCode())
        || l4HeightTunable.hasChanged(PivotConstants.class.hashCode())
        || prepareBargeHeightTunable.hasChanged(PivotConstants.class.hashCode())
        || bargeHeightTunable.hasChanged(PivotConstants.class.hashCode())
        || processorHeightTunable.hasChanged(PivotConstants.class.hashCode())) {
      L2Degrees = l2HeightTunable.get();
      L3Degrees = l3HeightTunable.get();
      L4Degrees = l4HeightTunable.get();
      prepareBargeDegrees = prepareBargeHeightTunable.get();
      bargeDegrees = bargeHeightTunable.get();
      processorDegrees = processorHeightTunable.get();
      lowAlgeaDegrees = lowAlgeaHeightTunable.get();
      highAlgeaDegrees = highAlgeaHeightTunable.get();
    }
  }
}
