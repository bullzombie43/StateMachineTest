// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AutoScoreConstants;
import frc.robot.autos.AutoFactory;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPose;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedSuperState;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOPhoenix6;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.endEffector.EndEffectorIO;
import frc.robot.subsystems.endEffector.EndEffectorIOPhoenix6;
import frc.robot.subsystems.endEffector.EndEffectorIOSim;
import frc.robot.subsystems.intake.AlgeaIntake;
import frc.robot.subsystems.intake.AlgeaIntakeIO;
import frc.robot.subsystems.intake.AlgeaIntakeIOPhoenix6;
import frc.robot.subsystems.intake.AlgeaIntakeIOSim;
import frc.robot.subsystems.intake.CoralIntake;
import frc.robot.subsystems.intake.CoralIntakeIO;
import frc.robot.subsystems.intake.CoralIntakeIOPhoenix6;
import frc.robot.subsystems.intake.CoralIntakeIOSim;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOPhoenix6;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.vision.*;
import frc.robot.util.MirroringUtil;
import java.util.List;
import java.util.Set;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private boolean isAlgea = false;
  private int coralScoringHeight = 1;
  public Pose2d targetPose = Pose2d.kZero;

  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Elevator elevator;
  private final Pivot pivot;
  private final AlgeaIntake algeaIntake;
  private final CoralIntake coralIntake;
  private final EndEffector endEffector;

  private final Superstructure superstructure;

  private SwerveDriveSimulation driveSimulation = null;

  // Controller
  private final CommandPS4Controller controller = new CommandPS4Controller(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Triggers
  private final Trigger hasCoral;
  private final Trigger isIntakingCoral;
  private final Trigger isIntakingAlgea;
  private final Trigger algeaMode;
  private final Trigger preparedProcesser;
  private final Trigger preparedBarge;
  private final Trigger coralPoleMode;
  private final Trigger coralTroughMode;
  private final Trigger isRemovingAlgea;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                new ModuleIOTalonFXReal(TunerConstants.BackRight),
                (pose) -> {});
        this.vision =
            new Vision(
                drive,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));

        elevator = new Elevator(new ElevatorIOPhoenix6());
        pivot = new Pivot(new PivotIOPhoenix6());
        coralIntake = new CoralIntake(new CoralIntakeIOPhoenix6());
        algeaIntake = new AlgeaIntake(new AlgeaIntakeIOPhoenix6());
        endEffector = new EndEffector(new EndEffectorIOPhoenix6());

        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations

        driveSimulation =
            new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);
        vision =
            new Vision(
                drive,
                new VisionIOPhotonVisionSim(
                    camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));

        pivot = new Pivot(new PivotIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        coralIntake = new CoralIntake(new CoralIntakeIOSim(driveSimulation));
        algeaIntake = new AlgeaIntake(new AlgeaIntakeIOSim(driveSimulation));
        endEffector = new EndEffector(new EndEffectorIOSim(driveSimulation));

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (pose) -> {});
        vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        pivot = new Pivot(new PivotIO() {});
        coralIntake = new CoralIntake(new CoralIntakeIO() {});
        algeaIntake = new AlgeaIntake(new AlgeaIntakeIO() {});
        endEffector = new EndEffector(new EndEffectorIO() {});

        break;
    }

    superstructure =
        new Superstructure(elevator, pivot, drive, coralIntake, algeaIntake, endEffector, this);

    // Set Up Triggers
    hasCoral = new Trigger(() -> coralIntake.hasCoral());
    isIntakingCoral =
        new Trigger(
            () -> superstructure.getWantedSuperState().equals(WantedSuperState.INTAKE_GROUND));
    isIntakingAlgea =
        new Trigger(
            () ->
                superstructure.getWantedSuperState().equals(WantedSuperState.ALGEA_GROUND_INTAKE));
    algeaMode = new Trigger(() -> isAlgea);
    preparedProcesser =
        new Trigger(
            () -> superstructure.getWantedSuperState().equals(WantedSuperState.PREPARE_PROCESSOR));
    preparedBarge =
        new Trigger(
            () -> superstructure.getWantedSuperState().equals(WantedSuperState.PREPARE_BARGE));
    coralPoleMode =
        new Trigger(
            () -> coralScoringHeight == 2 || coralScoringHeight == 3 || coralScoringHeight == 4);
    coralTroughMode = new Trigger(() -> coralScoringHeight == 1);
    isRemovingAlgea =
        new Trigger(
            () ->
                superstructure.getWantedSuperState().equals(WantedSuperState.REMOVE_HIGH_ALGEA)
                    || superstructure
                        .getWantedSuperState()
                        .equals(WantedSuperState.REMOVE_LOW_ALGEA));

    // Initialize AutoFactory
    AutoFactory.getInstance(this, DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("ProcEcho", AutoFactory.getInstance().procEcho());
    autoChooser.addOption("ProcEchoDelta", AutoFactory.getInstance().procEchoDelta());
    autoChooser.addOption("ProcEchoDeltaCharlie", AutoFactory.getInstance().procEchoDeltaCharlie());
    autoChooser.addDefaultOption(
        "ProcEchoDeltaCharlieBravo", AutoFactory.getInstance().procEchoDeltaCharlieBravo());
    autoChooser.addOption(
        "ProcEchoDeltaCharlieBravoL2", AutoFactory.getInstance().procEchoDeltaCharlieBravoL2());

    Logger.recordOutput("Poses/TargetPose", targetPose);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Switch to X pattern when X button is pressed
    controller.cross().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro / odometry
    final Runnable resetGyro =
        Constants.currentMode == Constants.Mode.SIM
            ? () -> drive.setPose(driveSimulation.getSimulatedDriveTrainPose()) // reset odometry to
            // actual robot pose
            // during simulation
            : () ->
                drive.setPose(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero
    // gyro
    controller.circle().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

    controller
        .R1()
        .and(algeaMode.negate())
        .and(coralPoleMode)
        .onTrue(scoreCoralPole().alongWith(new DriveToPose(drive, () -> getClosestRightPole())))
        .onFalse(
            superstructure.setWantedSuperState(Superstructure.WantedSuperState.STOW_ALL_SYSTEMS));

    controller
        .L1()
        .and(algeaMode.negate())
        .and(coralPoleMode)
        .onTrue(scoreCoralPole().alongWith(new DriveToPose(drive, () -> getClosestLeftPole())))
        .onFalse(
            superstructure.setWantedSuperState(Superstructure.WantedSuperState.STOW_ALL_SYSTEMS));

    controller
        .R1()
        .and(algeaMode)
        .onTrue(
            new ConditionalCommand(
                superstructure.setWantedSuperState(
                    Superstructure.WantedSuperState.PREPARE_PROCESSOR),
                superstructure.setWantedSuperState(Superstructure.WantedSuperState.SCORE_PROCESSOR),
                preparedProcesser.negate()));

    preparedProcesser.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> MirroringUtil.flipToCurrentAlliance(Rotation2d.kCCW_90deg)));

    controller
        .L1()
        .and(algeaMode)
        .onTrue(
            new ConditionalCommand(
                superstructure.setWantedSuperState(Superstructure.WantedSuperState.PREPARE_BARGE),
                superstructure.setWantedSuperState(Superstructure.WantedSuperState.SCORE_BARGE),
                preparedBarge.negate()));

    preparedBarge.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> MirroringUtil.flipToCurrentAlliance(Rotation2d.kZero)));

    controller
        .R2()
        .and(algeaMode.negate())
        .onTrue(
            new ConditionalCommand(
                superstructure.setWantedSuperState(WantedSuperState.INTAKE_GROUND),
                superstructure.setWantedSuperState(WantedSuperState.STOW_ALL_SYSTEMS),
                isIntakingCoral.negate()));
    controller
        .R2()
        .and(algeaMode)
        .onTrue(
            new ConditionalCommand(
                superstructure.setWantedSuperState(WantedSuperState.ALGEA_GROUND_INTAKE),
                superstructure.setWantedSuperState(WantedSuperState.STOW_ALL_SYSTEMS),
                isIntakingAlgea.negate()));
    controller
        .L2()
        .and(algeaMode)
        .onTrue(
            new ConditionalCommand(
                removeAlgeaCommand(),
                superstructure.setWantedSuperState(WantedSuperState.STOW_ALL_SYSTEMS),
                isRemovingAlgea.negate()));

    controller.share().and(algeaMode).onTrue(Commands.runOnce(() -> spawnAlgea()));
    controller.share().and(algeaMode.negate()).onTrue(Commands.runOnce(() -> spawnCoral()));

    controller
        .R3()
        .and(algeaMode.negate())
        .onTrue(superstructure.setWantedSuperState(Superstructure.WantedSuperState.OUTTAKE_CORAL));

    controller
        .R3()
        .and(algeaMode)
        .onTrue(superstructure.setWantedSuperState(Superstructure.WantedSuperState.OUTTAKE_ALGEA));

    controller.L3().onTrue(Commands.runOnce(() -> toggleAlgeaMode()));

    controller.povDown().onTrue(Commands.runOnce(() -> setCoralScoringHeight(1)));
    controller.povLeft().onTrue(Commands.runOnce(() -> setCoralScoringHeight(2)));
    controller.povRight().onTrue(Commands.runOnce(() -> setCoralScoringHeight(3)));
    controller.povUp().onTrue(Commands.runOnce(() -> setCoralScoringHeight(4)));

    controller
        .options()
        .onTrue(
            new DeferredCommand(
                () -> new DriveToPose(drive, () -> new Pose2d(3.0, 3.0, Rotation2d.kCW_90deg)),
                Set.of(drive)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();

    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }

  public void spawnCoral() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance()
        .addGamePiece(
            new ReefscapeCoralOnField(
                MirroringUtil.flipToCurrentAlliance(new Pose2d(3.5, 5, Rotation2d.kZero))));
  }

  public void spawnAlgea() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance()
        .addGamePiece(
            new ReefscapeAlgaeOnField(
                MirroringUtil.flipToCurrentAlliance(new Translation2d(3.5, 5))));
  }

  public CoralIntake getCoralIntake() {
    return coralIntake;
  }

  public AlgeaIntake getAlgeaIntake() {
    return algeaIntake;
  }

  public void setAlgeaMode(boolean isAlgea) {
    this.isAlgea = isAlgea;
  }

  public void toggleAlgeaMode() {
    isAlgea = !isAlgea;
    Logger.recordOutput("RobotContainer/isAlgea", isAlgea);
  }

  public void setCoralScoringHeight(int height) {
    this.coralScoringHeight = height;
  }

  public Command scoreCoralPole() {
    return superstructure.defer(
        () -> {
          if (coralScoringHeight == 2) {
            return superstructure.setWantedSuperState(WantedSuperState.SCORE_L2);
          } else if (coralScoringHeight == 3) {
            return superstructure.setWantedSuperState(WantedSuperState.SCORE_L3);
          } else {
            return superstructure.setWantedSuperState(WantedSuperState.SCORE_L4);
          }
        });
  }

  public Command removeAlgeaCommand() {
    return Commands.defer(
        () -> {
          Pose2d closestAlgeaPose =
              MirroringUtil.flipToCurrentAlliance(Constants.removeAlgeaPoses[0]);
          double closestDistance =
              drive.getPose().getTranslation().getDistance(closestAlgeaPose.getTranslation());
          int closestIndex = 0;

          for (int i = 0; i < Constants.removeAlgeaPoses.length; i++) {
            Pose2d pose = MirroringUtil.flipToCurrentAlliance(Constants.removeAlgeaPoses[i]);
            double distance = drive.getPose().getTranslation().getDistance(pose.getTranslation());
            if (distance < closestDistance) {
              closestAlgeaPose = pose;
              closestDistance = distance;
              closestIndex = i;
            }
          }

          Pose2d finalClosestAlgeaPose = closestAlgeaPose;

          Superstructure.WantedSuperState wantedState =
              (closestIndex % 2 == 0)
                  ? Superstructure.WantedSuperState.REMOVE_HIGH_ALGEA
                  : Superstructure.WantedSuperState.REMOVE_LOW_ALGEA;

          setTargetPose(finalClosestAlgeaPose);

          return (superstructure
              .setWantedSuperState(wantedState)
              .alongWith(new DriveToPose(drive, () -> finalClosestAlgeaPose))
              .until(algeaIntake::hasAlgea));
        },
        Set.of(superstructure, drive));
  }

  public Command scoreCoralTrough() {
    return superstructure.setWantedSuperState(WantedSuperState.SCORE_L1);
  }

  public Pose2d getClosestLeftPole() {
    List<Pose2d> poseList =
        AutoBuilder.shouldFlip()
            ? AutoScoreConstants.flippedLeftScorePoses
            : AutoScoreConstants.leftScorePoses;

    Pose2d closest = poseList.get(0);
    int closestNum = 0;

    for (int i = 1; i < poseList.size(); i++) {
      double currentDiff =
          Math.abs(poseList.get(i).getRotation().minus(drive.getRotation()).getRadians());
      double closestDiff = Math.abs(closest.getRotation().minus(drive.getRotation()).getRadians());

      if (currentDiff < closestDiff) {
        closest = poseList.get(i);
        closestNum = i;
      }
    }

    targetPose =
        MirroringUtil.flipToCurrentAlliance(AutoScoreConstants.leftScorePoses.get(closestNum));
    Logger.recordOutput("Poses/TargetPose", targetPose);

    return MirroringUtil.flipToCurrentAlliance(AutoScoreConstants.leftScorePoses.get(closestNum));
  }

  public Pose2d getClosestRightPole() {
    List<Pose2d> poseList =
        AutoBuilder.shouldFlip()
            ? AutoScoreConstants.flippedRightScorePoses
            : AutoScoreConstants.rightScorePoses;

    Pose2d closest = poseList.get(0);
    int closestNum = 0;

    for (int i = 1; i < poseList.size(); i++) {
      double currentDiff =
          Math.abs(poseList.get(i).getRotation().minus(drive.getRotation()).getRadians());
      double closestDiff = Math.abs(closest.getRotation().minus(drive.getRotation()).getRadians());

      if (currentDiff < closestDiff) {
        closest = poseList.get(i);
        closestNum = i;
      }
    }

    targetPose =
        MirroringUtil.flipToCurrentAlliance(AutoScoreConstants.rightScorePoses.get(closestNum));
    Logger.recordOutput("Poses/TargetPose", targetPose);

    return MirroringUtil.flipToCurrentAlliance(AutoScoreConstants.rightScorePoses.get(closestNum));
  }

  public void setTargetPose(Pose2d pose) {
    this.targetPose = pose;
    Logger.recordOutput("Poses/TargetPose", targetPose);
  }

  public Drive getSwerve() {
    return drive;
  }

  public Superstructure getSuperstructure() {
    return superstructure;
  }
}
