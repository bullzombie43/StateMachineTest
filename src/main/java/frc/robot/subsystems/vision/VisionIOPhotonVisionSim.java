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

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static VisionSystemSim visionSim;
  private static VisionSystemSim coralSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier, boolean coralCamera) {
    super(name, robotToCamera);
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
    }

    // Initialize coral sim
    if (coralSim == null) {
      coralSim = new VisionSystemSim("coral");
    }

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    cameraSim = new PhotonCameraSim(camera, cameraProperties, aprilTagLayout);
    if (coralCamera) {
      coralSim.addCamera(cameraSim, robotToCamera);
    } else {
      visionSim.addCamera(cameraSim, robotToCamera);
    }

    cameraSim.enableDrawWireframe(false);
    cameraSim.enableProcessedStream(false);
    cameraSim.enableRawStream(false);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    coralSim.update(poseSupplier.get());
    coralSim.clearVisionTargets();

    Pose3d[] coralPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Coral");
    VisionTargetSim[] targets = new VisionTargetSim[coralPoses.length];
    for (int i = 0; i < coralPoses.length; i++) {
      targets[i] = new VisionTargetSim(coralPoses[i], VisionConstants.CORAL_MODEL);
    }

    coralSim.addVisionTargets(targets);

    Logger.recordOutput("Vision/Coral/NumCoralTargets", targets.length);

    super.updateInputs(inputs);
  }
}
