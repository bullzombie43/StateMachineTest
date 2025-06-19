package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Drives to a specified pose. */
public class DriveToPose extends Command {
  private final ProfiledPIDController driveController;
  private final ProfiledPIDController thetaController;
  private Drive driveSubsystem;
  private Supplier<Pose2d> targetSupplier;
  private Translation2d lastSetpointTranslation;
  private double driveErrorAbs;
  private double thetaErrorAbs;
  private double ffMinRadius = 0.2;
  private double ffMaxRadius = 0.8;

  private final LoggedTunableNumber ffMinRadiusTunable =
      new LoggedTunableNumber("DriveToPose/ffMinRadius", ffMinRadius);
  private final LoggedTunableNumber ffMaxRadiusTunable =
      new LoggedTunableNumber("DriveToPose/ffMaxRadius", ffMaxRadius);
  private final LoggedTunableNumber driveKP =
      new LoggedTunableNumber("DriveToPose/translation/kP", 6.0);
  private final LoggedTunableNumber driveKI =
      new LoggedTunableNumber("DriveToPose/translation/kI", 0.0);
  private final LoggedTunableNumber driveKD =
      new LoggedTunableNumber("DriveToPose/translation/kD", 0.0);
  private final LoggedTunableNumber thetaKP =
      new LoggedTunableNumber("DriveToPose/rotation/kP", 4.0);
  private final LoggedTunableNumber thetaKI =
      new LoggedTunableNumber("DriveToPose/rotation/kI", 0.0);
  private final LoggedTunableNumber thetaKD =
      new LoggedTunableNumber("DriveToPose/rotation/kD", 0.0);
  private final LoggedTunableNumber driveMaxVelocity =
      new LoggedTunableNumber("DriveToPose/translation/maxVelocity", 0.0);
  private final LoggedTunableNumber driveMaxAcceleration =
      new LoggedTunableNumber("DriveToPose/translation/maxAcceleration", 0.0);
  private final LoggedTunableNumber thetaMaxVelocity =
      new LoggedTunableNumber("DriveToPose/rotation/maxVelocity", 0.0);
  private final LoggedTunableNumber thetaMaxAcceleration =
      new LoggedTunableNumber("DriveToPose/rotation/maxAcceleration", 0.0);
  private final LoggedTunableNumber driveErrorTolerance =
      new LoggedTunableNumber("DriveToPose/translation/errorTolerance", 0.02);
  private final LoggedTunableNumber thetaErrorTolerance =
      new LoggedTunableNumber("DriveToPose/rotation/errorTolerance", Units.degreesToRadians(2));

  public DriveToPose(Drive driveSubsystem, Supplier<Pose2d> targetSupplier) {
    driveController =
        new ProfiledPIDController(
            driveKP.get(),
            driveKI.get(),
            driveKD.get(),
            new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()));
    thetaController =
        new ProfiledPIDController(
            thetaKP.get(),
            thetaKI.get(),
            thetaKD.get(),
            new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()));
    driveController.setTolerance(driveErrorTolerance.get());
    thetaController.setTolerance(thetaErrorTolerance.get());

    this.driveSubsystem = driveSubsystem;
    this.targetSupplier = targetSupplier;
    addRequirements(driveSubsystem);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = driveSubsystem.getPose();
    ChassisSpeeds fieldVelocity = driveSubsystem.getChassisSpeeds();
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
    driveController.reset(
        currentPose.getTranslation().getDistance(targetSupplier.get().getTranslation()),
        Math.min(
            0.0,
            -linearFieldVelocity
                .rotateBy(
                    targetSupplier
                        .get()
                        .getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    thetaController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    lastSetpointTranslation = currentPose.getTranslation();
  }

  @Override
  public void execute() {
    checkIfGainsChanged();

    Pose2d currentPose = driveSubsystem.getPose();
    Pose2d targetPose = targetSupplier.get();

    Logger.recordOutput("DriveToPose/currentPose", currentPose);
    Logger.recordOutput("DriveToPose/targetPose", targetPose);

    double currentDistance =
        currentPose.getTranslation().getDistance(targetSupplier.get().getTranslation());
    double ffScaler =
        MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);
    driveErrorAbs = currentDistance;
    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);
    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(driveErrorAbs, 0.0);
    if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(
                GeomUtil.transform2dFromTranslation(
                    new Translation2d(driveController.getSetpoint().position, 0.0)))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        thetaController.getSetpoint().velocity * ffScaler
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

    // Command speeds
    var driveVelocity =
        GeomUtil.pose2dFromRotation(
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(
                GeomUtil.transform2dFromTranslation(new Translation2d(driveVelocityScalar, 0.0)))
            .getTranslation();
    driveSubsystem.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.runVelocity(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return targetSupplier.get().equals(null)
        || (driveController.atGoal() && thetaController.atGoal());
  }

  public void checkIfGainsChanged() {
    if (driveMaxVelocity.hasChanged(hashCode())
        || driveMaxAcceleration.hasChanged(hashCode())
        || driveErrorTolerance.hasChanged(hashCode())
        || thetaMaxVelocity.hasChanged(hashCode())
        || thetaMaxAcceleration.hasChanged(hashCode())
        || thetaErrorTolerance.hasChanged(hashCode())
        || driveKP.hasChanged(hashCode())
        || driveKD.hasChanged(hashCode())
        || thetaKP.hasChanged(hashCode())
        || thetaKD.hasChanged(hashCode())) {
      driveController.setP(driveKP.get());
      driveController.setD(driveKD.get());
      driveController.setConstraints(
          new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()));
      driveController.setTolerance(driveErrorTolerance.get());
      thetaController.setP(thetaKP.get());
      thetaController.setD(thetaKD.get());
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()));
      thetaController.setTolerance(thetaErrorTolerance.get());
    }
  }
}
