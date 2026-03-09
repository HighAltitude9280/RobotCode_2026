package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class VisionIOSim implements VisionIO {
  private final Supplier<Pose2d> poseSupplier;

  public VisionIOSim(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = true;
    Pose2d currentPose = poseSupplier.get();

    inputs.estimatedPose =
        new Pose2d(
            currentPose.getX() + (Math.random() - 0.5) * 0.02,
            currentPose.getY() + (Math.random() - 0.5) * 0.02,
            currentPose.getRotation());

    // Uso de Logger.getTimestamp() para determinismo en Replay/Sim
    inputs.timestamp = Logger.getTimestamp() / 1_000_000.0;

    inputs.targetCount = 2;
    inputs.averageTagDistance = 2.0;
    inputs.maxAmbiguity = 0.05;
    inputs.tagIds = new long[] {1, 2};
  }
}
