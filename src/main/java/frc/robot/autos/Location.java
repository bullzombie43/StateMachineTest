package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public enum Location {
  UNDEFINED,
  PROC_START,
  BARGE_START,
  ALPHA,
  BRAVO,
  CHARLIE,
  DELTA,
  ECHO,
  FOXTROT,
  GULF,
  HORSE,
  IGLOO,
  JOHNNY,
  KAYLA,
  LLAMA,
  PROC_INTAKE,
  BARGE_INTAKE;

  public static Pose2d getLocationPose(Location loc) {
    switch (loc) {
      case UNDEFINED:
        return Pose2d.kZero;
      case PROC_START:
        return new Pose2d(7.121, 1.310, Rotation2d.k180deg);
      case BARGE_START:
        return new Pose2d(7.133, 6.956, Rotation2d.k180deg);
      case ALPHA:
        return Constants.AutoScoreConstants.leftScorePoses.get(0);
      case BRAVO:
        return Constants.AutoScoreConstants.rightScorePoses.get(0);
      case CHARLIE:
        return Constants.AutoScoreConstants.leftScorePoses.get(1);
      case DELTA:
        return Constants.AutoScoreConstants.rightScorePoses.get(1);
      case ECHO:
        return Constants.AutoScoreConstants.leftScorePoses.get(2);
      case FOXTROT:
        return Constants.AutoScoreConstants.rightScorePoses.get(2);
      case GULF:
        return Constants.AutoScoreConstants.leftScorePoses.get(3);
      case HORSE:
        return Constants.AutoScoreConstants.rightScorePoses.get(3);
      case IGLOO:
        return Constants.AutoScoreConstants.leftScorePoses.get(4);
      case JOHNNY:
        return Constants.AutoScoreConstants.rightScorePoses.get(4);
      case KAYLA:
        return Constants.AutoScoreConstants.leftScorePoses.get(5);
      case LLAMA:
        return Constants.AutoScoreConstants.rightScorePoses.get(5);
      case PROC_INTAKE:
        return new Pose2d(1.765, 0.955, Rotation2d.fromDegrees(-129));
      case BARGE_INTAKE:
        return new Pose2d(1.765, 7.097, Rotation2d.fromDegrees(129));
      default:
        return Pose2d.kZero; // Default case if no match found
    }
  }
}
