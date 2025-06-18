// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffector;

import frc.robot.Constants;
import frc.robot.util.Gains;

/** Add your docs here. */
public class EndEffectorConstants {
  public static final int endEffectorMotorID = 17;
  public static final String endEffectorCanbus = "3045 Canivore";

  public static Gains gains =
      switch (Constants.currentMode) {
        case SIM -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case REAL -> new Gains(0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case REPLAY -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      };

  public static final double endEffectorSupplyCurrentLimit = 40.0;
  public static final double endEffectorStatorCurrentLimit = 80.0;

  public static final double coralOuttakeVel = 10.0; // Rotations Per Second
  public static final double coralIntakeVel = -10.0; // Rotations Per Second
  public static final double algeaOuttakeVel = 10.0;
  public static final double algeaIntakeVel = -10.0;
}
