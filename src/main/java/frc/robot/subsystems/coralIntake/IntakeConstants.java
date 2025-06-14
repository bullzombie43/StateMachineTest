// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coralIntake;

import frc.robot.Constants;
import frc.robot.util.Gains;

/** Add your docs here. */
public class IntakeConstants {
    public static final int coralIntakePivotID = 1;
    public static final String coralIntakeCanbus = "3045 Canivore";

    public static Gains coralPivotGains = 
        switch (Constants.currentMode) {
            case SIM -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            case REAL -> new Gains(0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            case REPLAY -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        };
    
    public static final double coralPivotSupplyCurrentLimit = 40.0; 
    public static final double coralPivotStatorCurrentLimit = 80.0;   
    
}
