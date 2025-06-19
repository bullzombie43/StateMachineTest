package frc.robot.commands;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autos.AutoSegment;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class FollowPathCommand extends Command {
  /** Timer object */
  private final Timer timer = new Timer();

  /** Drivetrain object to access subsystem. */
  private final Drive swerveSubsystem;

  /** {@link Pair} to follow. */
  private final AutoSegment segment;

  /** {@link PathPlannerTrajectory} to follow */
  private PathPlannerTrajectory trajectory;

  private static final PIDController rotationPID;

  private static final PIDController translationPID;

  private static final LoggedTunableNumber kRotationP =
      new LoggedTunableNumber("PathFollowing/RotationP", 0.5);
  private static final LoggedTunableNumber kRotationI =
      new LoggedTunableNumber("PathFollowing/RotationI", 0.0);
  private static final LoggedTunableNumber kRotationD =
      new LoggedTunableNumber("PathFollowing/RotationD", 0.0);
  private static final LoggedTunableNumber kTranslationP =
      new LoggedTunableNumber("PathFollowing/TranslationP", 0.5);
  private static final LoggedTunableNumber kTranslationI =
      new LoggedTunableNumber("PathFollowing/TranslationI", 0.0);
  private static final LoggedTunableNumber kTranslationD =
      new LoggedTunableNumber("PathFollowing/TranslationD", 0.0);

  public FollowPathCommand(final AutoSegment segment, Drive swerveSubsystem) {
    this.segment = segment;
    this.swerveSubsystem = swerveSubsystem;

    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    rotationPID.reset();
    translationPID.reset();

    trajectory =
        new PathPlannerTrajectory(
            segment.path(),
            swerveSubsystem.getChassisSpeeds(),
            swerveSubsystem.getRotation(),
            Drive.PP_CONFIG);

    Logger.recordOutput("PathFollowing/PathName", segment.name());
    Logger.recordOutput("PathFollowing/PerceivedOrientation", swerveSubsystem.getRotation());
    this.timer.restart();
  }

  @Override
  public void execute() {
    double currentTime = this.timer.get();
    // Determine desired state based on where the robot should be at the current
    // time in the path
    PathPlannerTrajectoryState desiredState = trajectory.sample(currentTime);
    var currentPose = swerveSubsystem.getPose();

    Rotation2d heading = desiredState.heading;

    // Calculate our target velocity based on current pose and desired state
    var vx = desiredState.linearVelocity * Math.cos(heading.getRadians());
    var vy = desiredState.linearVelocity * Math.sin(heading.getRadians());
    var desiredThetaSpeeds =
        rotationPID.calculate(
            currentPose.getRotation().getRadians(), desiredState.heading.getRadians());

    double xFeedback = translationPID.calculate(currentPose.getX(), desiredState.pose.getX());
    double yFeedback = translationPID.calculate(currentPose.getY(), desiredState.pose.getY());

    ChassisSpeeds chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            vx + xFeedback, vy + yFeedback, desiredThetaSpeeds, currentPose.getRotation());

    swerveSubsystem.runVelocity(chassisSpeeds);

    Logger.recordOutput("PathFollowing/DesiredStatePose", desiredState.pose);
    Logger.recordOutput("PathFollowing/DesiredChassisSpeeds", chassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    this.timer.stop(); // Stop timer
    swerveSubsystem.runVelocity(new ChassisSpeeds()); // Stop motors
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }

  public Pose2d getStart() {
    return trajectory.getInitialState().pose;
  }

  static {
    translationPID =
        new PIDController(kTranslationP.get(), kTranslationI.get(), kTranslationD.get());
    rotationPID = new PIDController(kRotationP.get(), kRotationI.get(), kRotationD.get());
    rotationPID.enableContinuousInput(-Math.PI, Math.PI);
  }
}
