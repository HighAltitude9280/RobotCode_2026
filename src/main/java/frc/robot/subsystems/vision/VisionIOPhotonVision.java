package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.TAG_LAYOUT;

import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhotonVision implements VisionIO {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;

  public VisionIOPhotonVision(String cameraName, Transform3d robotToCamera) {
    camera = new PhotonCamera(cameraName);

    poseEstimator =
        new PhotonPoseEstimator(
            TAG_LAYOUT,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCamera);

    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Reset base cada loop
    inputs.connected = camera.isConnected();
    inputs.hasEstimate = false;
    inputs.timestampSeconds = 0.0;
    inputs.tagCount = 0;
    inputs.averageTagDistanceMeters = 0.0;
    inputs.ambiguity = 0.0;
    inputs.seenTagIds = new int[] {};

    List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults();
    if (unreadResults.isEmpty()) {
      return;
    }

    // Resultado más reciente de este loop
    PhotonPipelineResult result = unreadResults.get(unreadResults.size() - 1);

    inputs.timestampSeconds = result.getTimestampSeconds();

    if (!result.hasTargets()) {
      return;
    }

    var targets = result.getTargets();
    inputs.tagCount = targets.size();

    int[] seenIds = new int[targets.size()];
    double totalDistance = 0.0;
    double maxAmbiguity = 0.0;

    for (int i = 0; i < targets.size(); i++) {
      PhotonTrackedTarget target = targets.get(i);

      seenIds[i] = target.getFiducialId();
      totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();

      double ambiguity = target.getPoseAmbiguity();
      if (ambiguity > maxAmbiguity) {
        maxAmbiguity = ambiguity;
      }
    }

    inputs.seenTagIds = seenIds;
    inputs.averageTagDistanceMeters = totalDistance / targets.size();
    inputs.ambiguity = maxAmbiguity;

    var estimate = poseEstimator.update(result);
    if (estimate.isPresent()) {
      EstimatedRobotPose estimatedPose = estimate.get();
      inputs.hasEstimate = true;
      inputs.estimatedRobotPose = estimatedPose.estimatedPose;
      inputs.timestampSeconds = estimatedPose.timestampSeconds;
    }
  }
}