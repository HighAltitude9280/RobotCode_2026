package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public double timestamp = 0.0;
    public Pose2d estimatedPose = new Pose2d();
    public int targetCount = 0;
    public double averageTagDistance = 0.0;
    public long[] tagIds = new long[] {};
    public double maxAmbiguity = 0.0;
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
