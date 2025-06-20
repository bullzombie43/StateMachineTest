// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.util.MirroringUtil;
import java.util.Set;
import java.util.function.Supplier;

/** Add your docs here. */
public class AutoFactory {
  private final RobotContainer robotContainer;
  private final Drive swerve;
  private final Superstructure superstructure;
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
  }

  public Command procEchoDelta() {
    AutoSegment echoIntake = loadSegment(Location.ECHO, Location.PROC_INTAKE, "Echo-Intake");

    preloadTrajectoryClass(echoIntake);

    SequentialCommandGroup group = new SequentialCommandGroup();

    group.addCommands(resetPose(Location.PROC_START));
    group.addCommands(driveThenScoreL4(Location.ECHO));
    group.addCommands(Commands.waitSeconds(0.1));
    group.addCommands(segmentUntilGroundIntake(echoIntake));
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

  public Command driveThenScoreL4(Location scoringLoc) {
    return Commands.parallel(
        superstructure.setWantedSuperState(WantedSuperState.SCORE_L4), driveToPoint(scoringLoc));
  }

  public Command stowAllSubsystems() {
    return superstructure.setWantedSuperState(WantedSuperState.STOW_ALL_SYSTEMS);
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
    // This is done because Java loads classes lazily. Calling this here loads the trajectory class
    // which
    // is used to follow paths and saves user code ms loop time at the start of auto.
    if (!trajectoriesLoaded) {
      trajectoriesLoaded = true;
      var trajectory =
          new PathPlannerTrajectory(
              segment.path(), swerve.getChassisSpeeds(), swerve.getRotation(), Drive.PP_CONFIG);
    }
  }
}
