package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

public class MirroringUtil {
  public static final double FIELD_WIDTH = 17.548;
  public static final double FIELD_HEIGHT = 8.052;

  public static boolean isRedSide() {
    final Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get().equals(DriverStation.Alliance.Red);
  }

  public static Rotation2d flipRotation2d(Rotation2d rotation) {
    return rotation.plus(Rotation2d.k180deg);
  }

  public static Translation2d flipTranslation2d(Translation2d translation) {
    return new Translation2d(FIELD_WIDTH - translation.getX(), FIELD_HEIGHT - translation.getY());
  }

  public static Translation3d flipTranslation3d(Translation3d translation3d) {
    final Translation2d flippedTranslation2d = flipTranslation2d(translation3d.toTranslation2d());
    return new Translation3d(
        flippedTranslation2d.getX(), flippedTranslation2d.getY(), translation3d.getZ());
  }

  public static Rotation3d flipRotation3d(Rotation3d rotation3d) {
    final Rotation2d flippedRotation2d = flipRotation2d(rotation3d.toRotation2d());
    return new Rotation3d(
        flippedRotation2d.getRadians(), flippedRotation2d.getRadians(), rotation3d.getZ());
  }

  public static Pose2d flip(Pose2d pose) {
    return new Pose2d(flipTranslation2d(pose.getTranslation()), flipRotation2d(pose.getRotation()));
  }

  public static Pose3d flip(Pose3d pose) {
    return new Pose3d(flipTranslation3d(pose.getTranslation()), flipRotation3d(pose.getRotation()));
  }

  public static Translation2d flipToCurrentAlliance(Translation2d translationAtBlueSide) {
    return isRedSide() ? flipTranslation2d(translationAtBlueSide) : translationAtBlueSide;
  }

  public static Rotation2d flipToCurrentAlliance(Rotation2d rotationAtBlueSide) {
    return isRedSide() ? flipRotation2d(rotationAtBlueSide) : rotationAtBlueSide;
  }

  public static Pose2d flipToCurrentAlliance(Pose2d poseAtBlueSide) {
    return new Pose2d(
        flipToCurrentAlliance(poseAtBlueSide.getTranslation()),
        flipToCurrentAlliance(poseAtBlueSide.getRotation()));
  }

  public static Pose2d[] fliptoCurrentAlliance(Pose2d[] posesAtBlueSide) {
    Pose2d[] flippedPoses = new Pose2d[posesAtBlueSide.length];
    for (int i = 0; i < posesAtBlueSide.length; i++) {
      flippedPoses[i] = flipToCurrentAlliance(posesAtBlueSide[i]);
    }
    return flippedPoses;
  }

  public static Translation3d flipToCurrentAlliance(Translation3d translation3dAtBlueSide) {
    final Translation2d translation3dAtCurrentAlliance =
        flipToCurrentAlliance(translation3dAtBlueSide.toTranslation2d());
    if (isRedSide())
      return new Translation3d(
          translation3dAtCurrentAlliance.getX(),
          translation3dAtCurrentAlliance.getY(),
          translation3dAtBlueSide.getZ());
    return translation3dAtBlueSide;
  }

  public static Rotation3d flipToCurrentAlliance(Rotation3d rotation3dAtBlueSide) {
    final Rotation2d rotation2dAtCurrentAlliance =
        flipToCurrentAlliance(rotation3dAtBlueSide.toRotation2d());
    if (isRedSide())
      return new Rotation3d(
          rotation2dAtCurrentAlliance.getRadians(),
          rotation2dAtCurrentAlliance.getRadians(),
          rotation3dAtBlueSide.getZ());
    return rotation3dAtBlueSide;
  }

  public static final Pose3d flipToCurrentAlliance(Pose3d pose) {
    return new Pose3d(
        flipToCurrentAlliance(pose.getTranslation()), flipToCurrentAlliance(pose.getRotation()));
  }

  public static final Pose3d[] flipToCurrentAlliance(Pose3d[] posesAtBlueSide) {
    Pose3d[] flippedPoses = new Pose3d[posesAtBlueSide.length];
    for (int i = 0; i < posesAtBlueSide.length; i++) {
      flippedPoses[i] = flipToCurrentAlliance(posesAtBlueSide[i]);
    }
    return flippedPoses;
  }
}
