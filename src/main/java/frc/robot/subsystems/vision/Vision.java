package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

  @FunctionalInterface
  public interface VisionMeasurementConsumer {
    void accept(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs);
  }

  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final VisionMeasurementConsumer visionConsumer;

  public Vision(VisionMeasurementConsumer visionConsumer, VisionIO... io) {
    this.visionConsumer = visionConsumer;
    this.io = io;

    inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);

      boolean accepted = shouldAcceptMeasurement(inputs[i]);
      Logger.recordOutput("Vision/Camera" + i + "/Accepted", accepted);

      if (!accepted) {
        continue;
      }

      Pose2d pose2d = inputs[i].estimatedRobotPose.toPose2d();
      Matrix<N3, N1> stdDevs = getStdDevs(inputs[i]);

      visionConsumer.accept(
          pose2d,
          inputs[i].timestampSeconds,
          stdDevs);

      Logger.recordOutput("Vision/Camera" + i + "/RobotPose2d", pose2d);
      Logger.recordOutput("Vision/Camera" + i + "/StdDevs", stdDevs.getData());
    }
  }

  private boolean shouldAcceptMeasurement(VisionIOInputsAutoLogged input) {
    if (!input.connected) {
      return false;
    }

    if (!input.hasEstimate) {
      return false;
    }

    if (input.tagCount <= 0) {
      return false;
    }

    if (input.averageTagDistanceMeters > MAX_TAG_DISTANCE) {
      return false;
    }

    if (input.tagCount == 1 && input.ambiguity > MAX_SINGLE_TAG_AMBIGUITY) {
      return false;
    }

    Pose2d pose = input.estimatedRobotPose.toPose2d();

    if (pose.getX() < 0.0 || pose.getX() > TAG_LAYOUT.getFieldLength()) {
      return false;
    }

    if (pose.getY() < 0.0 || pose.getY() > TAG_LAYOUT.getFieldWidth()) {
      return false;
    }

    return true;
  }

  private Matrix<N3, N1> getStdDevs(VisionIOInputsAutoLogged input) {
    double xyStdDev =
        BASE_XY_STD_DEV
            * Math.pow(input.averageTagDistanceMeters, 2.0)
            / Math.max(input.tagCount, 1);

    double thetaStdDev;
    if (input.tagCount >= 2) {
      thetaStdDev =
          BASE_THETA_STD_DEV
              * Math.pow(input.averageTagDistanceMeters, 2.0)
              / input.tagCount;
    } else {
      thetaStdDev = Double.MAX_VALUE;
    }

    return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
  }
}