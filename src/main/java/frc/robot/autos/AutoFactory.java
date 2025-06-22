// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.FollowAlliancePathCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedSuperState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.GeomUtil;
import frc.robot.util.MirroringUtil;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class AutoFactory {
  private final RobotContainer robotContainer;
  private final Drive swerve;
  private final Superstructure superstructure;
  private final Vision vision;
  private final DriverStation.Alliance alliance;

  private boolean trajectoriesLoaded = false;

  private static AutoFactory instance;

  public static AutoFactory getInstance(
      RobotContainer robotContainer, DriverStation.Alliance alliance) {
    if (instance == null) {
      instance = new AutoFactory(robotContainer, alliance);
    }
    return instance;
  }

  public static AutoFactory getInstance() {
    if (instance == null) {
      throw new IllegalStateException(
          "AutoFactory has not been initialized. Call getInstance(RobotContainer, DriverStation.Alliance) first.");
    }
    return instance;
  }

  public AutoFactory(RobotContainer robotContainer, DriverStation.Alliance alliance) {
    this.robotContainer = robotContainer;
    this.swerve = robotContainer.getSwerve();
    this.superstructure = robotContainer.getSuperstructure();
    this.alliance = alliance;
    this.vision = robotContainer.getVision();
  }

  public Command procEchoDeltaCharlieBravoL2() {

    SequentialCommandGroup group = new SequentialCommandGroup();

    group.addCommands(resetPose(Location.PROC_START));
    group.addCommands(driveThenScoreL4(Location.ECHO));
    group.addCommands(Commands.waitSeconds(0.1));
    group.addCommands(driveUntilGroundIntake(Location.PROC_INTAKE));
    group.addCommands(driveThenScoreL4(Location.DELTA));
    group.addCommands(Commands.waitSeconds(0.1));
    group.addCommands(driveUntilGroundIntake(Location.PROC_INTAKE));
    group.addCommands(driveThenScoreL4(Location.CHARLIE));
    group.addCommands(Commands.waitSeconds(0.1));
    group.addCommands(driveUntilGroundIntake(Location.PROC_INTAKE));
    group.addCommands(driveThenScoreL4(Location.BRAVO));
    group.addCommands(Commands.waitSeconds(0.1));
    group.addCommands(driveUntilGroundIntake(Location.MIDDLE_CORALALGEA));
    group.addCommands(driveThenScoreL2(Location.BRAVO));
    group.addCommands(Commands.waitSeconds(0.1));
    group.addCommands(stowAllSubsystems());

    return group;
  }

  public Command procEchoDeltaCharlieBravo() {
    SequentialCommandGroup group = new SequentialCommandGroup();

    group.addCommands(resetPose(Location.PROC_START));
    group.addCommands(driveThenScoreL4(Location.ECHO));
    group.addCommands(Commands.waitSeconds(0.1));
    group.addCommands(driveUntilGroundIntake(Location.PROC_INTAKE));
    group.addCommands(driveThenScoreL4(Location.DELTA));
    group.addCommands(Commands.waitSeconds(0.1));
    group.addCommands(driveUntilGroundIntake(Location.PROC_INTAKE));
    group.addCommands(driveThenScoreL4(Location.CHARLIE));
    group.addCommands(Commands.waitSeconds(0.1));
    group.addCommands(driveUntilGroundIntake(Location.PROC_INTAKE));
    group.addCommands(driveThenScoreL4(Location.BRAVO));
    group.addCommands(Commands.waitSeconds(0.1));
    group.addCommands(stowAllSubsystems());

    return group;
  }

  public Command procEchoDeltaCharlie() {

    SequentialCommandGroup group = new SequentialCommandGroup();

    group.addCommands(resetPose(Location.PROC_START));
    group.addCommands(driveThenScoreL4(Location.ECHO));
    group.addCommands(Commands.waitSeconds(0.1));
    group.addCommands(driveUntilGroundIntake(Location.PROC_INTAKE));
    group.addCommands(driveThenScoreL4(Location.DELTA));
    group.addCommands(Commands.waitSeconds(0.1));
    group.addCommands(driveUntilGroundIntake(Location.PROC_INTAKE));
    group.addCommands(driveThenScoreL4(Location.CHARLIE));
    group.addCommands(Commands.waitSeconds(0.1));
    group.addCommands(stowAllSubsystems());

    return group;
  }

  public Command procEchoDelta() {
    SequentialCommandGroup group = new SequentialCommandGroup();

    group.addCommands(resetPose(Location.PROC_START));
    group.addCommands(driveThenScoreL4(Location.ECHO));
    group.addCommands(Commands.waitSeconds(0.1));
    group.addCommands(driveUntilGroundIntake(Location.PROC_INTAKE));
    group.addCommands(driveThenScoreL4(Location.DELTA));
    group.addCommands(Commands.waitSeconds(0.1));
    group.addCommands(stowAllSubsystems());

    return group;
  }

  public Command procEcho() {
    SequentialCommandGroup group = new SequentialCommandGroup();

    group.addCommands(resetPose(Location.PROC_START));
    group.addCommands(driveThenScoreL4(Location.ECHO));
    group.addCommands(Commands.waitSeconds(0.1));
    group.addCommands(stowAllSubsystems());

    return group;
  }

  public Command segmentThenScoreL4(Location start, Location end, String name) {
    return Commands.parallel(
        superstructure.setWantedSuperState(WantedSuperState.SCORE_L4),
        followSegment(loadSegment(start, end, name)));
  }

  public Command driveThenScoreL2(Location scoringLoc) {
    return Commands.parallel(
        superstructure.setWantedSuperState(WantedSuperState.SCORE_L2), driveToPoint(scoringLoc));
  }

  public Command driveThenScoreL3(Location scoringLoc) {
    return Commands.parallel(
        superstructure.setWantedSuperState(WantedSuperState.SCORE_L3), driveToPoint(scoringLoc));
  }

  public Command driveThenScoreL4(Location scoringLoc) {
    return Commands.parallel(
        superstructure.setWantedSuperState(WantedSuperState.SCORE_L4), driveToPoint(scoringLoc));
  }

  public Command stowAllSubsystems() {
    return superstructure.setWantedSuperState(WantedSuperState.STOW_ALL_SYSTEMS);
  }

  public Command driveUntilGroundIntake(Location loc) {
    return Commands.parallel(
            superstructure.setWantedSuperState(WantedSuperState.INTAKE_GROUND),
            driveWithCoralDetection(loc))
        .until(() -> superstructure.hasCoral());
  }

  public Command segmentUntilGroundIntake(AutoSegment segment) {
    return Commands.parallel(
            superstructure.setWantedSuperState(WantedSuperState.INTAKE_GROUND),
            followSegment(segment))
        .until(() -> superstructure.hasCoral());
  }

  public Command driveToPoint(Location loc) {
    return driveToPoint(() -> MirroringUtil.flipToCurrentAlliance(Location.getLocationPose(loc)));
  }

  public Command driveToPoint(Supplier<Pose2d> point) {
    return Commands.defer(
        () ->
            Commands.runOnce(() -> robotContainer.setTargetPose(point.get()))
                .andThen(new DriveToPose(swerve, point)),
        Set.of(swerve));
  }

  public Command resetPose(AutoSegment segment) {
    return resetPose(segment.path().getStartingHolonomicPose().orElse(swerve.getPose()));
  }

  public Command followSegment(AutoSegment segment) {
    return new FollowAlliancePathCommand(segment, swerve);
  }

  public Command resetPose(Location location) {
    return resetPose(Location.getLocationPose(location));
  }

  public Command resetPose(Pose2d pose) {
    return new DeferredCommand(
        () -> new InstantCommand(() -> swerve.setPose(MirroringUtil.flipToCurrentAlliance(pose))),
        Set.of());
  }

  public Command driveWithCoralDetection(Location loc) {
    return driveWithCoralDetection(Location.getLocationPose(loc));
  }

  public Command driveWithCoralDetection(Pose2d originalTargetPose) {
    Supplier<Pose2d> poseSupplier =
        new Supplier<>() {
          private Pose2d lastSeenPose = originalTargetPose;
          private long lastSeenTime = System.currentTimeMillis();

          @Override
          public Pose2d get() {
            Rotation2d tX = vision.getTargetX(2);
            Rotation2d tY = vision.getTargetY(2);

            if (tX.getDegrees() == 0 && tY.getDegrees() == 0) {
              // No target detected
              Logger.recordOutput("DriveToPose/seenCoral", true);
              if (System.currentTimeMillis() - lastSeenTime <= 500) {
                return lastSeenPose; // Use last seen pose for 0.5 seconds
              }

              return originalTargetPose; // Fallback to original pose
            }

            Double distance =
                (VisionConstants.CAMERA_HEIGHT - VisionConstants.CORAL_HEIGHT)
                    / Math.tan(VisionConstants.CAMERA_ANGLE + tY.getDegrees());

            Pose2d cameraPose =
                swerve
                    .getPose()
                    .transformBy(GeomUtil.transform3dTo2d(VisionConstants.robotToCoralCamera));

            Transform2d cameraToCoral =
                new Transform2d(
                    new Translation2d(Math.abs(distance), 0.0).rotateBy(tX.unaryMinus()),
                    tX.unaryMinus());

            Pose2d coralPose = cameraPose.transformBy(cameraToCoral);

            Logger.recordOutput("DriveToPose/distance", distance);
            Logger.recordOutput("DriveToPose/tX", tX.getDegrees());
            Logger.recordOutput("DriveToPose/seenCoral", true);

            // Update last seen pose and time
            lastSeenPose = coralPose;
            lastSeenTime = System.currentTimeMillis();

            return coralPose;
          }
        };

    return new DriveToPose(swerve, poseSupplier).until(() -> superstructure.hasCoral());
  }

  private AutoSegment loadSegment(final Location start, final Location end, String name) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(name);
      return new AutoSegment(start, end, name, path);
    } catch (Exception e) {
      System.out.println("Failed to load path for segment: " + start + " to " + end);
      e.printStackTrace(); // Log the stack trace for debugging purposes
      return null; // Return null to indicate the segment could not be loaded
    }
  }

  private void preloadTrajectoryClass(AutoSegment segment) {
    // This is done because Java loads classes lazily. Calling this here loads the
    // trajectory class
    // which
    // is used to follow paths and saves user code ms loop time at the start of
    // auto.
    if (!trajectoriesLoaded) {
      trajectoriesLoaded = true;
      var trajectory =
          new PathPlannerTrajectory(
              segment.path(), swerve.getChassisSpeeds(), swerve.getRotation(), Drive.PP_CONFIG);
    }
  }
}
