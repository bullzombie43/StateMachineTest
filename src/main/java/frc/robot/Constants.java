// Copyright 2021-2025 FRC 6328
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

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final boolean tuningMode = true;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  public static Pose3d[] algeaPoses =
      new Pose3d[] {
        new Pose3d(),
        new Pose3d(),
        new Pose3d(),
        new Pose3d(),
        new Pose3d(),
        new Pose3d(),
        new Pose3d(),
        new Pose3d(),
        new Pose3d(),
      };

  public static Pose3d[] netAlgeaPoses =
      new Pose3d[] {
        new Pose3d(8.785, 6.146 - 1.6, 2.1, Rotation3d.kZero),
        new Pose3d(8.785, 6.146 - 1.2, 2.1, Rotation3d.kZero),
        new Pose3d(8.785, 6.146 - 0.8, 2.1, Rotation3d.kZero),
        new Pose3d(8.785, 6.146 - 0.4, 2.1, Rotation3d.kZero),
        new Pose3d(8.785, 6.146, 2.1, Rotation3d.kZero),
        new Pose3d(8.785, 6.146 + 0.4, 2.1, Rotation3d.kZero),
        new Pose3d(8.785, 6.146 + 0.8, 2.1, Rotation3d.kZero),
        new Pose3d(8.785, 6.146 + 1.2, 2.1, Rotation3d.kZero),
        new Pose3d(8.785, 6.146 + 1.6, 2.1, Rotation3d.kZero),
      };

  public class AutoScoreConstants {
    public static HashMap<Integer, Pose2d> kScorePoseMap = new HashMap<Integer, Pose2d>();

    public static List<Pose2d> rightScorePoses = new ArrayList<Pose2d>();
    public static List<Pose2d> leftScorePoses = new ArrayList<Pose2d>();
    public static List<Pose2d> flippedRightScorePoses = new ArrayList<>();
    public static List<Pose2d> flippedLeftScorePoses = new ArrayList<>();

    static {
      kScorePoseMap.put(1, new Pose2d(3.18, 4.18, Rotation2d.k180deg));
      kScorePoseMap.put(2, new Pose2d(3.17, 3.850, Rotation2d.k180deg));
      kScorePoseMap.put(3, new Pose2d(3.69, 2.91, Rotation2d.fromDegrees(-120)));
      kScorePoseMap.put(4, new Pose2d(3.97, 2.76, Rotation2d.fromDegrees(-120)));
      kScorePoseMap.put(5, new Pose2d(5.002, 2.828, Rotation2d.fromDegrees(-60)));
      kScorePoseMap.put(6, new Pose2d(5.32, 2.94, Rotation2d.fromDegrees(-60)));
      kScorePoseMap.put(7, new Pose2d(5.85, 3.87, Rotation2d.kZero));
      kScorePoseMap.put(8, new Pose2d(5.85, 4.21, Rotation2d.kZero));
      kScorePoseMap.put(9, new Pose2d(5.30, 5.1, Rotation2d.fromDegrees(60)));
      kScorePoseMap.put(10, new Pose2d(5.011, 5.26, Rotation2d.fromDegrees(60)));
      kScorePoseMap.put(11, new Pose2d(3.97, 5.26, Rotation2d.fromDegrees(120)));
      kScorePoseMap.put(12, new Pose2d(3.66, 5.08, Rotation2d.fromDegrees(120)));

      List<Integer> sortedKeys = new ArrayList<>(kScorePoseMap.keySet());
      Collections.sort(sortedKeys);

      for (Integer key : sortedKeys) {
        if (key % 2 == 0) {
          rightScorePoses.add(kScorePoseMap.get(key));
        } else {
          leftScorePoses.add(kScorePoseMap.get(key));
        }

        flippedRightScorePoses =
            rightScorePoses.stream().map((pose) -> FlippingUtil.flipFieldPose(pose)).toList();
        flippedLeftScorePoses =
            leftScorePoses.stream().map((pose) -> FlippingUtil.flipFieldPose(pose)).toList();
      }
    }
  }
}
