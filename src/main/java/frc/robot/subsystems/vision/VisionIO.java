package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public boolean hasEstimate = false;
    public Pose3d estimatedRobotPose = new Pose3d();
    public double timestampSeconds = 0.0;
    public int tagCount = 0;
    public double averageTagDistanceMeters = 0.0;
    public double ambiguity = 0.0;
    public int[] seenTagIds = new int[] {};
  }

  default void updateInputs(VisionIOInputs inputs) {}

  default void setRecording(boolean active) {}
}