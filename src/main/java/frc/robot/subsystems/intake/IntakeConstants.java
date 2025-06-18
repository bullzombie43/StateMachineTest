// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import frc.robot.util.Gains;

/** Add your docs here. */
public class IntakeConstants {
  public static final int coralIntakePivotID = 13;
  public static final int algeaIntakePivotID = 14;
  public static final int coralIntakeRollerID = 15;
  public static final int algeaIntakeRollerID = 16;
  public static final String coralIntakeCanbus = "3045 Canivore";
  public static final String algeaIntakeCanbus = "3045 Canivore";

  public static Gains coralPivotGains =
      switch (Constants.currentMode) {
        case SIM -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case REAL -> new Gains(0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case REPLAY -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      };
  public static Gains algeaPivotGains =
      switch (Constants.currentMode) {
        case SIM -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case REAL -> new Gains(0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case REPLAY -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      };
  public static Gains coralRollerGains =
      switch (Constants.currentMode) {
        case SIM -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case REAL -> new Gains(0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case REPLAY -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      };
  public static Gains algeaRollerGains =
      switch (Constants.currentMode) {
        case SIM -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case REAL -> new Gains(0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case REPLAY -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      };

  public static final double coralPivotSupplyCurrentLimit = 40.0;
  public static final double coralPivotStatorCurrentLimit = 80.0;
  public static final double algeaPivotSupplyCurrentLimit = 40.0;
  public static final double algeaPivotStatorCurrentLimit = 80.0;
  public static final double coralRollerSupplyCurrentLimit = 40.0;
  public static final double coralRollerStatorCurrentLimit = 80.0;
  public static final double algeaRollerSupplyCurrentLimit = 40.0;
  public static final double algeaRollerStatorCurrentLimit = 80.0;

  public static final double acceptablePitchErrorDegrees = 5.0;

  public static final double maxAngleDegrees = 90.0;
  public static final double minAngleDegrees = 0.0;

  public static final double coralStowDegrees = 0.0;
  public static final double coralIntakeDegrees = 90.0;

  public static final double algeaStowDegrees = 0.0;
  public static final double algeaIntakeDegrees = 45.0;

  public static final double coralRollerForwardSpeed = 10.0; // Rotations Per Second
  public static final double coralRollerReverseSpeed = -10.0; // Rotations Per Second
  public static final double algeaRollerForwardSpeed = 10.0; // Rotations Per Second
  public static final double algeaRollerReverseSpeed = -10.0; // Rotations Per Second

  public static final Transform3d coralIntakeOffset =
      new Transform3d(-0.13, 0, 0.23, new Rotation3d());
  public static final Transform3d algeaIntakeOffset =
      new Transform3d(0.18, -0.13, 0.2, new Rotation3d());
}
