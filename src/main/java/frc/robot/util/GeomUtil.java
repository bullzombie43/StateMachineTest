package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;

/** Geometry utilities for working with translations, rotations, transforms, and poses. */
public class GeomUtil {
  /**
   * Creates a pure translating transform
   *
   * @param translation The translation to create the transform with
   * @return The resulting transform
   */
  public static Transform2d translationToTransform(Translation2d translation) {
    return new Transform2d(translation, new Rotation2d());
  }

  /**
   * Creates a pure translating transform
   *
   * @param x The x componenet of the translation
   * @param y The y componenet of the translation
   * @return The resulting transform
   */
  public static Transform2d toTransform2d(double x, double y) {
    return new Transform2d(new Translation2d(x, y), new Rotation2d());
  }

  /**
   * Creates a pure rotating transform
   *
   * @param rotation The rotation to create the transform with
   * @return The resulting transform
   */
  public static Transform2d rotationToTransform(Rotation2d rotation) {
    return new Transform2d(new Translation2d(), rotation);
  }

  /**
   * Converts a Pose2d to a Transform2d to be used in a kinematic chain
   *
   * @param pose The pose that will represent the transform
   * @return The resulting transform
   */
  public static Transform2d poseToTransform(Pose2d pose) {
    return new Transform2d(pose.getTranslation(), pose.getRotation());
  }

  /**
   * Converts a Transform2d to a Pose2d to be used as a position or as the start of a kinematic
   * chain
   *
   * @param transform The transform that will represent the pose
   * @return The resulting pose
   */
  public static Pose2d transformToPose(Transform2d transform) {
    return new Pose2d(transform.getTranslation(), transform.getRotation());
  }

  /**
   * Creates a pure translated pose
   *
   * @param translation The translation to create the pose with
   * @return The resulting pose
   */
  public static Pose2d translationToPose(Translation2d translation) {
    return new Pose2d(translation, new Rotation2d());
  }

  /**
   * Creates a pure rotated pose
   *
   * @param rotation The rotation to create the pose with
   * @return The resulting pose
   */
  public static Pose2d rotationToPose(Rotation2d rotation) {
    return new Pose2d(new Translation2d(), rotation);
  }

  /**
   * Multiplies a twist by a scaling factor
   *
   * @param twist The twist to multiply
   * @param factor The scaling factor for the twist components
   * @return The new twist
   */
  public static Twist2d multiplyTwist(Twist2d twist, double factor) {
    return new Twist2d(twist.dx * factor, twist.dy * factor, twist.dtheta * factor);
  }

  /**
   * Converts a Pose3d to a Transform3d to be used in a kinematic chain
   *
   * @param pose The pose that will represent the transform
   * @return The resulting transform
   */
  public static Transform3d pose3dToTransform3d(Pose3d pose) {
    return new Transform3d(pose.getTranslation(), pose.getRotation());
  }

  /**
   * Converts a Transform3d to a Pose3d to be used as a position or as the start of a kinematic
   * chain
   *
   * @param transform The transform that will represent the pose
   * @return The resulting pose
   */
  public static Pose3d transform3dToPose3d(Transform3d transform) {
    return new Pose3d(transform.getTranslation(), transform.getRotation());
  }

  /**
   * Converts a Translation3d to a Translation2d by extracting two dimensions (X and Y). chain
   *
   * @param transform The original translation
   * @return The resulting translation
   */
  public static Translation2d translation3dTo2dXY(Translation3d translation) {
    return new Translation2d(translation.getX(), translation.getY());
  }

  /**
   * Converts a Translation3d to a Translation2d by extracting two dimensions (X and Z). chain
   *
   * @param transform The original translation
   * @return The resulting translation
   */
  public static Translation2d translation3dTo2dXZ(Translation3d translation) {
    return new Translation2d(translation.getX(), translation.getZ());
  }

  /**
   * Creates a 3d pure translating transform
   *
   * @param translation The translation to create the transform with
   * @return The resulting transform
   */
  public static Transform3d translation3dToTransform(Translation3d translation) {
    return new Transform3d(translation, new Rotation3d());
  }

  /**
   * Check if poses are close to each other Currently only checks their translations
   *
   * @param targetPose Targetted Pose
   * @param actualPose Actual Pose
   * @param tolerance tolerance to be considered close enough
   * @return whether or not the poses are near each other
   */
  public static boolean isNearPose(Pose2d targetPose, Pose2d actualPose, double tolerance) {
    return targetPose.getTranslation().getDistance(actualPose.getTranslation()) < tolerance;
  }

  public static boolean isNearPoses(Pose2d[] targetPoses, Pose2d actualPose, double tolerance) {
    for (Pose2d targetPose : targetPoses) {
      if (isNearPose(targetPose, actualPose, tolerance)) {
        return true;
      }
    }
    return false;
  }

  /**
   * Check if poses are close to each other Checks both rotation and translation
   *
   * @param targetPose Targetted Pose
   * @param actualPose Actual Pose
   * @param translationToleranceMeters tolerance to be considered close enough in meters
   * @param rotationToleranceDegrees tolerance to be considered close enough in Degrees
   * @return whether or not the poses are near each other
   */
  public static boolean isNearPoseWithRotation(
      Pose2d targetPose,
      Pose2d actualPose,
      double translationToleranceMeters,
      double rotationToleranceDegrees) {
    return targetPose.getTranslation().getDistance(actualPose.getTranslation())
            < translationToleranceMeters
        && Math.abs(targetPose.getRotation().minus(actualPose.getRotation()).getDegrees())
            < rotationToleranceDegrees;
  }

  public static final Transform2d transform2dFromTranslation(Translation2d translation) {
    return new Transform2d(translation, Rotation2d.kZero);
  }

  public static final Pose2d pose2dFromRotation(Rotation2d rotation) {
    return new Pose2d(Translation2d.kZero, rotation);
  }

  public static final Pose2d pose2dFromTranslation(Translation2d translation) {
    return new Pose2d(translation, Rotation2d.kZero);
  }

  public static final Transform2d transform3dTo2d(Transform3d transform) {
    return new Transform2d(
        new Translation2d(transform.getX(), transform.getY()),
        transform.getRotation().toRotation2d());
  }
}
