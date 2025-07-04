// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.AlgeaIntake;
import frc.robot.subsystems.intake.CoralIntake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.util.GeomUtil;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  public static Pose3d coralPose = new Pose3d();
  public static Pose3d algeaPose = new Pose3d();

  private Elevator elevator;
  private Pivot pivot;
  private CoralIntake coralIntake;
  private AlgeaIntake algeaIntake;
  private EndEffector endEffector;
  private Drive drivetrain;
  private RobotContainer robotContainer;

  public enum WantedSuperState {
    HOME_ALL_SYSTEMS,
    SCORE_L1,
    SCORE_L2,
    SCORE_L3,
    SCORE_L4,
    SCORE_L4_REMOVE_LOW_ALGEA,
    SCORE_L4_REMOVE_HIGH_ALGEA,
    REMOVE_HIGH_ALGEA,
    REMOVE_LOW_ALGEA,
    INTAKE_GROUND,
    INTAKE_SUBSTATION,
    STOW_ALL_SYSTEMS,
    PREPARE_PROCESSOR,
    SCORE_PROCESSOR,
    PREPARE_BARGE,
    SCORE_BARGE,
    ALGEA_GROUND_INTAKE,
    OUTTAKE_CORAL,
    OUTTAKE_ALGEA,
    STOPPED
  }

  public enum CurrentSuperState {
    HOMING_ALL_SYSTEMS,
    SCORING_L1,
    SCORING_L2,
    SCORING_L3,
    SCORING_L4,
    SCORING_L4_REMOVE_LOW_ALGEA,
    SCORING_L4_REMOVE_HIGH_ALGEA,
    REMOVING_HIGH_ALGEA,
    REMOVING_LOW_ALGEA,
    INTAKING_CORAL_GROUND,
    INTAKING_SUBSTATION,
    STOWING_ALL_SYSTEMS,
    PREPARING_PROCESSOR,
    SCORING_PROCESSOR,
    PREPARING_BARGE,
    SCORING_BARGE,
    INTAKING_ALGEA_GROUND,
    OUTTAKING_CORAL,
    OUTTAKING_ALGEA,
    STOPPED
  }

  private WantedSuperState wantedSuperState = WantedSuperState.STOW_ALL_SYSTEMS;
  private CurrentSuperState currentSuperState = CurrentSuperState.STOWING_ALL_SYSTEMS;
  private CurrentSuperState previousSuperState = CurrentSuperState.STOWING_ALL_SYSTEMS;

  public Superstructure(
      Elevator elevator,
      Pivot pivot,
      Drive drive,
      CoralIntake coralIntake,
      AlgeaIntake algeaIntake,
      EndEffector endEffector,
      RobotContainer robotContainer) {
    this.elevator = elevator;
    this.pivot = pivot;
    this.drivetrain = drive;
    this.coralIntake = coralIntake;
    this.algeaIntake = algeaIntake;
    this.endEffector = endEffector;
    this.robotContainer = robotContainer;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    currentSuperState = handleStateTranstion();
    applyCurrentState();

    Logger.recordOutput("DesiredSuperstate", wantedSuperState);
    if (currentSuperState != previousSuperState) {
      Logger.recordOutput("CurrentSuperstate", currentSuperState);
    }
  }

  public CurrentSuperState handleStateTranstion() {
    previousSuperState = currentSuperState;
    switch (wantedSuperState) {
      case ALGEA_GROUND_INTAKE:
        currentSuperState = CurrentSuperState.INTAKING_ALGEA_GROUND;
        break;
      case HOME_ALL_SYSTEMS:
        currentSuperState = CurrentSuperState.HOMING_ALL_SYSTEMS;
        break;
      case INTAKE_GROUND:
        currentSuperState = CurrentSuperState.INTAKING_CORAL_GROUND;
        break;
      case INTAKE_SUBSTATION:
        currentSuperState = CurrentSuperState.INTAKING_SUBSTATION;
        break;
      case PREPARE_BARGE:
        currentSuperState = CurrentSuperState.PREPARING_BARGE;
        break;
      case PREPARE_PROCESSOR:
        currentSuperState = CurrentSuperState.PREPARING_PROCESSOR;
        break;
      case REMOVE_HIGH_ALGEA:
        currentSuperState = CurrentSuperState.REMOVING_HIGH_ALGEA;
        break;
      case REMOVE_LOW_ALGEA:
        currentSuperState = CurrentSuperState.REMOVING_LOW_ALGEA;
        break;
      case SCORE_BARGE:
        currentSuperState = CurrentSuperState.SCORING_BARGE;
        break;
      case SCORE_L1:
        currentSuperState = CurrentSuperState.SCORING_L1;
        break;
      case SCORE_L2:
        currentSuperState = CurrentSuperState.SCORING_L2;
        break;
      case SCORE_L3:
        currentSuperState = CurrentSuperState.SCORING_L3;
        break;
      case SCORE_L4:
        currentSuperState = CurrentSuperState.SCORING_L4;
        break;
      case SCORE_L4_REMOVE_HIGH_ALGEA:
        currentSuperState = CurrentSuperState.SCORING_L4_REMOVE_HIGH_ALGEA;
        break;
      case SCORE_L4_REMOVE_LOW_ALGEA:
        currentSuperState = CurrentSuperState.SCORING_L4_REMOVE_LOW_ALGEA;
        break;
      case SCORE_PROCESSOR:
        currentSuperState = CurrentSuperState.SCORING_PROCESSOR;
        break;
      case STOW_ALL_SYSTEMS:
        currentSuperState = CurrentSuperState.STOWING_ALL_SYSTEMS;
        break;
      case OUTTAKE_CORAL:
        currentSuperState = CurrentSuperState.OUTTAKING_CORAL;
        break;
      case OUTTAKE_ALGEA:
        currentSuperState = CurrentSuperState.OUTTAKING_ALGEA;
        break;
      case STOPPED:
      default:
        currentSuperState = CurrentSuperState.STOPPED;
        break;
    }

    return currentSuperState;
  }

  private void applyCurrentState() {
    switch (currentSuperState) {
      case HOMING_ALL_SYSTEMS:
        homeAllSystems();
        break;
      case INTAKING_ALGEA_GROUND:
        intakeGroundAlgea();
        break;
      case INTAKING_CORAL_GROUND:
        intakeGroundCoral();
        break;
      case INTAKING_SUBSTATION:
        intakeSubstation();
        break;
      case PREPARING_BARGE:
        prepareBarge();
        break;
      case PREPARING_PROCESSOR:
        prepareProcessor();
        break;
      case REMOVING_HIGH_ALGEA:
        removeHighAlgea();
        break;
      case REMOVING_LOW_ALGEA:
        removeLowAlgea();
        break;
      case SCORING_PROCESSOR:
        scoreProcessor();
        break;
      case SCORING_BARGE:
        scoreBarge();
        break;
      case SCORING_L1:
        scoreL1();
        break;
      case SCORING_L2:
        scoreL2();
        break;
      case SCORING_L3:
        scoreL3();
        break;
      case SCORING_L4:
        scoreL4();
        break;
      case SCORING_L4_REMOVE_HIGH_ALGEA:
        scoreL4RemoveHighAlgea();
        break;
      case SCORING_L4_REMOVE_LOW_ALGEA:
        scoreL4RemoveLowAlgea();
        break;
      case STOWING_ALL_SYSTEMS:
        stowAllSystems();
        break;
      case OUTTAKING_CORAL:
        outtakeCoral();
        break;
      case OUTTAKING_ALGEA:
        outtakeAlgea();
        break;
      case STOPPED:
      default:
        handleStopped();
        break;
    }
  }

  private void homeAllSystems() {
    // Logic to home all systems
  }

  private void intakeGroundAlgea() {
    // Logic to intake ground algae
    elevator.setWantedStateFunc(Elevator.WantedState.ALGEA_INTAKE);
    pivot.setWantedStateFunc(Pivot.WantedState.ALGEA_INTAKE);
    coralIntake.setWantedStateFunc(CoralIntake.WantedState.OUT_NO_INTAKE);
    algeaIntake.setWantedStateFunc(AlgeaIntake.WantedState.OUT_NO_INTAKE);

    if (pivot.atSetpoint() && elevator.atSetpoint()) {
      algeaIntake.setWantedStateFunc(AlgeaIntake.WantedState.INTAKE);
    }

    if (algeaIntake.hasAlgea()) setWantedSuperStateFunc(WantedSuperState.STOW_ALL_SYSTEMS);
  }

  private void intakeGroundCoral() {
    // Logic to intake ground coral
    elevator.setWantedStateFunc(Elevator.WantedState.INTAKE);
    pivot.setWantedStateFunc(Pivot.WantedState.INTAKE);
    coralIntake.setWantedStateFunc(CoralIntake.WantedState.OUT_NO_INTAKE);
    algeaIntake.setWantedStateFunc(AlgeaIntake.WantedState.OUT_NO_INTAKE);
    endEffector.setWantedStateFunc(EndEffector.WantedState.INTAKE);

    if (pivot.atSetpoint() && elevator.atSetpoint()) {
      coralIntake.setWantedStateFunc(CoralIntake.WantedState.INTAKE);
    }

    if (coralIntake.hasCoral()) setWantedSuperStateFunc(WantedSuperState.STOW_ALL_SYSTEMS);
  }

  private void intakeSubstation() {
    // Logic to intake from substation
  }

  private void prepareProcessor() {
    // Logic to prepare the processor
    coralIntake.setWantedStateFunc(CoralIntake.WantedState.OUT_NO_INTAKE);
    algeaIntake.setWantedStateFunc(AlgeaIntake.WantedState.OUT_NO_INTAKE);
    elevator.setWantedStateFunc(Elevator.WantedState.PROCESSOR);
    pivot.setWantedStateFunc(Pivot.WantedState.PROCESSOR);

    if (elevator.atSetpoint() && pivot.atSetpoint()) {
      coralIntake.setWantedStateFunc(CoralIntake.WantedState.STOW);
      algeaIntake.setWantedStateFunc(AlgeaIntake.WantedState.PREPARE_PROCESSOR);
    }
  }

  private void scoreProcessor() {
    // Logic to score using the processor
    algeaIntake.setWantedStateFunc(AlgeaIntake.WantedState.PROCESSOR);

    if (!algeaIntake.hasAlgea()) {
      algeaIntake.setWantedStateFunc(AlgeaIntake.WantedState.OUT_NO_INTAKE);
      coralIntake.setWantedStateFunc(CoralIntake.WantedState.OUT_NO_INTAKE);

      if (algeaIntake.atPivotSetpoint() && coralIntake.atPivotSetpoint()) {
        elevator.setWantedStateFunc(Elevator.WantedState.STOW);
        pivot.setWantedStateFunc(Pivot.WantedState.STOW);

        if (elevator.atSetpoint() && pivot.atSetpoint()) {
          setWantedSuperStateFunc(WantedSuperState.STOW_ALL_SYSTEMS);
        }
      }
    }
  }

  private void removeHighAlgea() {
    // Logic to remove high algae
    elevator.setWantedStateFunc(Elevator.WantedState.HIGH_ALGEA);
    pivot.setWantedStateFunc(Pivot.WantedState.HIGH_ALGEA);

    if (algeaIntake.hasAlgea()) {
      setWantedSuperStateFunc(WantedSuperState.STOW_ALL_SYSTEMS);
    }

    if (elevator.atSetpoint() && pivot.atSetpoint()) {
      coralIntake.setWantedStateFunc(CoralIntake.WantedState.STOW);
      algeaIntake.setWantedStateFunc(AlgeaIntake.WantedState.STOW);
      endEffector.setWantedStateFunc(EndEffector.WantedState.INTAKE);
    }
  }

  private void removeLowAlgea() {
    // Logic to remove low algae
    elevator.setWantedStateFunc(Elevator.WantedState.LOW_ALGEA);
    pivot.setWantedStateFunc(Pivot.WantedState.LOW_ALGEA);

    if (algeaIntake.hasAlgea()) {
      setWantedSuperStateFunc(WantedSuperState.STOW_ALL_SYSTEMS);
    }

    if (elevator.atSetpoint() && pivot.atSetpoint()) {
      coralIntake.setWantedStateFunc(CoralIntake.WantedState.STOW);
      algeaIntake.setWantedStateFunc(AlgeaIntake.WantedState.STOW);
      endEffector.setWantedStateFunc(EndEffector.WantedState.INTAKE);
    }
  }

  private void prepareBarge() {
    // Logic to prepare the barge
    coralIntake.setWantedStateFunc(CoralIntake.WantedState.STOW);
    algeaIntake.setWantedStateFunc(AlgeaIntake.WantedState.STOW);
    elevator.setWantedStateFunc(Elevator.WantedState.BARGE);
    pivot.setWantedStateFunc(Pivot.WantedState.BARGE);

    // Lock Rotation
  }

  private void scoreBarge() {
    // Logic to score using the barge
    elevator.setWantedStateFunc(Elevator.WantedState.BARGE);
    pivot.setWantedStateFunc(Pivot.WantedState.BARGE);
    endEffector.setWantedStateFunc(EndEffector.WantedState.SHOOT_ALGEA);

    if (!endEffector.hasAlgea()) {
      setWantedSuperStateFunc(WantedSuperState.STOW_ALL_SYSTEMS);
    }
  }

  private void scoreL1() {
    // Logic to score at level 1
    elevator.setWantedStateFunc(Elevator.WantedState.L1);
    pivot.setWantedStateFunc(Pivot.WantedState.L1);

    // DRIVING TO POSE IS DONE OUTSIDE OF SUPERSTRUCTURE AS A PARRALLEL COMMAND
    // DEFINED IN ROBOT
    // CONTAINER

    // CHECK IF ELEVATOR AND PIVOT ARE AT THE RIGHT HEIGHT
    if (elevator.atSetpoint() && pivot.atSetpoint()) {
      coralIntake.setWantedStateFunc(CoralIntake.WantedState.STOW);
      algeaIntake.setWantedStateFunc(AlgeaIntake.WantedState.STOW);

      // CHECK IF WE ARE IN RIGHT POSITION

      // PLACEHOLDER: OUTTAKE THE PIECE, THIS CAN MAYBE JUST TURN ROLLERS ON AND A
      // DIFFERENT STATE
      // WILL TURN THEM OFF WHEN THE ARM STOWS
      endEffector.setWantedStateFunc(EndEffector.WantedState.OUTTAKE);
    }
  }

  private void scoreL2() {
    // Logic to score at level 2
    elevator.setWantedStateFunc(Elevator.WantedState.L2);
    pivot.setWantedStateFunc(Pivot.WantedState.L2);

    // DRIVING TO POSE IS DONE OUTSIDE OF SUPERSTRUCTURE AS A PARRALLEL COMMAND
    // DEFINED IN ROBOT
    // CONTAINER

    // CHECK IF ELEVATOR AND PIVOT ARE AT THE RIGHT HEIGHT
    if (elevator.atSetpoint() && pivot.atSetpoint()) {
      coralIntake.setWantedStateFunc(CoralIntake.WantedState.STOW);
      algeaIntake.setWantedStateFunc(AlgeaIntake.WantedState.STOW);

      // CHECK IF WE ARE IN RIGHT POSITION
      if (GeomUtil.isNearPose(robotContainer.targetPose, drivetrain.getPose(), 0.02)) {
        // PLACEHOLDER: OUTTAKE THE PIECE, THIS CAN MAYBE JUST TURN ROLLERS ON AND A
        // DIFFERENT STATE
        // WILL TURN THEM OFF WHEN THE ARM STOWS
        endEffector.setWantedStateFunc(EndEffector.WantedState.OUTTAKE);
      }
    }
  }

  private void scoreL3() {
    // Logic to score at level 3
    elevator.setWantedStateFunc(Elevator.WantedState.L3);
    pivot.setWantedStateFunc(Pivot.WantedState.L3);

    // DRIVING TO POSE IS DONE OUTSIDE OF SUPERSTRUCTURE AS A PARRALLEL COMMAND
    // DEFINED IN ROBOT
    // CONTAINER

    // CHECK IF ELEVATOR AND PIVOT ARE AT THE RIGHT HEIGHT
    if (elevator.atSetpoint() && pivot.atSetpoint()) {
      coralIntake.setWantedStateFunc(CoralIntake.WantedState.STOW);
      algeaIntake.setWantedStateFunc(AlgeaIntake.WantedState.STOW);

      // CHECK IF WE ARE IN RIGHT POSITION
      if (GeomUtil.isNearPose(robotContainer.targetPose, drivetrain.getPose(), 0.02)) {
        // PLACEHOLDER: OUTTAKE THE PIECE, THIS CAN MAYBE JUST TURN ROLLERS ON AND A
        // DIFFERENT STATE
        // WILL TURN THEM OFF WHEN THE ARM STOWS
        endEffector.setWantedStateFunc(EndEffector.WantedState.OUTTAKE);
      }
    }
  }

  private void scoreL4() {
    // Logic to score at level 4
    elevator.setWantedStateFunc(Elevator.WantedState.L4);
    pivot.setWantedStateFunc(Pivot.WantedState.L4);

    // DRIVING TO POSE IS DONE OUTSIDE OF SUPERSTRUCTURE AS A PARRALLEL COMMAND
    // DEFINED IN ROBOT
    // CONTAINER

    // CHECK IF ELEVATOR AND PIVOT ARE AT THE RIGHT HEIGHT
    if (elevator.atSetpoint() && pivot.atSetpoint()) {
      coralIntake.setWantedStateFunc(CoralIntake.WantedState.STOW);
      algeaIntake.setWantedStateFunc(AlgeaIntake.WantedState.STOW);

      // CHECK IF WE ARE IN RIGHT POSITION
      if (GeomUtil.isNearPose(robotContainer.targetPose, drivetrain.getPose(), 0.02)) {
        // PLACEHOLDER: OUTTAKE THE PIECE, THIS CAN MAYBE JUST TURN ROLLERS ON AND A
        // DIFFERENT STATE
        // WILL TURN THEM OFF WHEN THE ARM STOWS
        endEffector.setWantedStateFunc(EndEffector.WantedState.OUTTAKE);
      }
    }
  }

  private void scoreL4RemoveHighAlgea() {
    // Logic to score at level 4 and remove high algae
  }

  private void scoreL4RemoveLowAlgea() {
    // Logic to score at level 4 and remove low algae
  }

  private void stowAllSystems() {
    // Logic to stow all systems
    elevator.setWantedStateFunc(Elevator.WantedState.STOW);
    pivot.setWantedStateFunc(Pivot.WantedState.STOW);
    endEffector.setWantedStateFunc(EndEffector.WantedState.STOPPED);

    if (previousSuperState == CurrentSuperState.INTAKING_CORAL_GROUND || algeaIntake.hasAlgea())
      coralIntake.setWantedStateFunc(CoralIntake.WantedState.OUT_NO_INTAKE);

    if (elevator.atSetpoint() && pivot.atSetpoint()) {
      // If the elevator and pivot are at their stow positions, we can also stow the
      // coral intake

      if (algeaIntake.hasAlgea())
        coralIntake.setWantedStateFunc(CoralIntake.WantedState.OUT_NO_INTAKE);
      else coralIntake.setWantedStateFunc(CoralIntake.WantedState.STOW);
      algeaIntake.setWantedStateFunc(AlgeaIntake.WantedState.STOW);
    }
  }

  private void outtakeCoral() {
    elevator.setWantedStateFunc(Elevator.WantedState.AlGEA_OUTTAKE);
    pivot.setWantedStateFunc(Pivot.WantedState.AlGEA_OUTTAKE);

    if (pivot.atSetpoint() && elevator.atSetpoint()) {
      coralIntake.setWantedStateFunc(CoralIntake.WantedState.REVERSING);
    }

    if (!coralIntake.hasCoral()) setWantedSuperStateFunc(WantedSuperState.STOW_ALL_SYSTEMS);
  }

  private void outtakeAlgea() {
    elevator.setWantedStateFunc(Elevator.WantedState.AlGEA_OUTTAKE);
    pivot.setWantedStateFunc(Pivot.WantedState.AlGEA_OUTTAKE);

    if (pivot.atSetpoint() && elevator.atSetpoint()) {
      algeaIntake.setWantedStateFunc(AlgeaIntake.WantedState.OUTTAKE);
    }

    if (!algeaIntake.hasAlgea()) setWantedSuperStateFunc(WantedSuperState.STOW_ALL_SYSTEMS);
  }

  private void handleStopped() {
    // Logic to handle stopped state
  }

  private void setWantedSuperStateFunc(WantedSuperState wantedSuperState) {
    this.wantedSuperState = wantedSuperState;
  }

  public Command setWantedSuperState(WantedSuperState wantedSuperState) {
    return runOnce(() -> setWantedSuperStateFunc(wantedSuperState));
  }

  public WantedSuperState getWantedSuperState() {
    return wantedSuperState;
  }

  public CurrentSuperState getCurrentSuperState() {
    return currentSuperState;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // You can add simulation-specific logic here if needed

    if (coralIntake.hasCoral()) {
      coralPose =
          new Pose3d(drivetrain.getPose())
              .transformBy(GeomUtil.pose3dToTransform3d(Robot.componentPoses[3]))
              .transformBy(IntakeConstants.coralIntakeOffset);
    } else {
      coralPose = Pose3d.kZero;
    }

    if (algeaIntake.hasAlgea()) {
      algeaPose =
          new Pose3d(drivetrain.getPose())
              .transformBy(GeomUtil.pose3dToTransform3d(Robot.componentPoses[3]))
              .transformBy(IntakeConstants.algeaIntakeOffset);
    } else {
      algeaPose = Pose3d.kZero;
    }

    Logger.recordOutput("Poses/IntakeCoral", coralPose);
    Logger.recordOutput("Poses/IntakeAlgea", algeaPose);

    if ((currentSuperState == CurrentSuperState.REMOVING_HIGH_ALGEA
            || currentSuperState == CurrentSuperState.REMOVING_LOW_ALGEA)
        && GeomUtil.isNearPose(robotContainer.targetPose, drivetrain.getPose(), 0.075)
        && elevator.atSetpoint()
        && pivot.atSetpoint()
        && !algeaIntake.hasAlgea()) {

      int closestPoseIndex = 0;
      double closestDistance =
          Constants.reefAlgeaPoses[0]
              .getTranslation()
              .toTranslation2d()
              .getDistance(drivetrain.getPose().getTranslation());

      for (int i = 0; i < Constants.reefAlgeaPoses.length; i++) {
        double distance =
            Constants.reefAlgeaPoses[i]
                .getTranslation()
                .toTranslation2d()
                .getDistance(drivetrain.getPose().getTranslation());

        if (distance < closestDistance) {
          closestDistance = distance;
          closestPoseIndex = i;
        }
      }

      if (closestDistance < 1.0) {
        algeaIntake.setHasAlgea(true);
        Constants.reefAlgeaPoses[closestPoseIndex] = Pose3d.kZero;
        Logger.recordOutput("Poses/ReefAlgea", Constants.reefAlgeaPoses);
      }
    }
  }

  public boolean hasCoral() {
    return coralIntake.hasCoral();
  }
}
