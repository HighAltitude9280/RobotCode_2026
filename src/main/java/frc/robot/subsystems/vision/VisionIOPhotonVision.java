package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.TAG_LAYOUT;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhotonVision implements VisionIO {
  private final PhotonCamera camera;
  private final Transform3d robotToCamera;

  public VisionIOPhotonVision(String cameraName, Transform3d robotToCamera) {
    this.camera = new PhotonCamera(cameraName);
    this.robotToCamera = robotToCamera;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();
    if (!inputs.connected)
      return;

    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (results.isEmpty())
      return;

    PhotonPipelineResult result = results.get(results.size() - 1);

    if (!result.hasTargets()) {
      inputs.targetCount = 0;
      return;
    }

    var targets = result.getTargets();
    inputs.targetCount = targets.size();
    inputs.tagIds = targets.stream().mapToLong(t -> (long) t.getFiducialId()).toArray();

    double totalDist = 0;
    double maxAmb = 0;
    for (PhotonTrackedTarget target : targets) {
      totalDist += target.getBestCameraToTarget().getTranslation().getNorm();
      maxAmb = Math.max(maxAmb, target.getPoseAmbiguity());
    }
    inputs.averageTagDistance = totalDist / targets.size();
    inputs.maxAmbiguity = maxAmb;

    // Extracción directa y cálculo manual de Pose sin depender de métodos
    // deprecados
    var multiTagResultOpt = result.getMultiTagResult();

    if (multiTagResultOpt.isPresent()) {
      // El coprocesador ya resolvió PNP para múltiples tags
      Transform3d fieldToCamera = multiTagResultOpt.get().estimatedPose.best;
      Pose3d cameraPose = new Pose3d().plus(fieldToCamera);

      // Aplicamos el offset inverso usando transformBy (Estándar WPILib)
      Pose3d robotPose = cameraPose.transformBy(robotToCamera.inverse());

      inputs.estimatedPose = robotPose.toPose2d();
      inputs.timestamp = result.getTimestampSeconds();

    } else {
      // Fallback robusto: Si hay 1 tag, o si Multi-Tag falló pero vemos varios tags,
      // confiamos en el tag más grande/mejor (índice 0).
      PhotonTrackedTarget bestTarget = targets.get(0);
      var tagPoseOpt = TAG_LAYOUT.getTagPose(bestTarget.getFiducialId());

      if (tagPoseOpt.isPresent()) {
        Pose3d tagPose = tagPoseOpt.get();
        Pose3d cameraPose = tagPose.transformBy(bestTarget.getBestCameraToTarget().inverse());
        Pose3d robotPose = cameraPose.transformBy(robotToCamera.inverse());

        inputs.estimatedPose = robotPose.toPose2d();
        inputs.timestamp = result.getTimestampSeconds();
      }
    }
  }
}