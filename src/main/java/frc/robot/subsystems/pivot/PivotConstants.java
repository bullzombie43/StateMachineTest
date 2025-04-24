// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import frc.robot.Constants;
import frc.robot.util.Gains;

/** Add your docs here. */
public class PivotConstants {
  public static final int pivotMotorId = 1;
  public static final int pivotEncoderId = 2;
  public static final String pivotCanbus = "3045 Canivore";

  public static final double PIVOT_GEAR_RATIO = (36.0 / 9.0) * (40.0 / 14.0) * (64.0 / 12.0);
  public static final double MOTOR_ROTATIONS_PER_DEGREE = PIVOT_GEAR_RATIO / 360.0;
  public static final double DEGREES_PER_ROTATION = 1.0 / MOTOR_ROTATIONS_PER_DEGREE;

  public static final double supplyCurrentLimit = 20.0;
  public static final double statorCurrentLimit = 80.0;

  public static final double magnetOffset = Constants.currentMode == Constants.simMode ? 0.0 : 0.0;

  public static Gains gains =
      switch (Constants.currentMode) {
        case SIM -> new Gains(0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case REAL -> new Gains(0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case REPLAY -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      };
  public static double maxVelocityRotPerSec = 1;
  public static double maxAccelerationRotPerSec = 2;
}
