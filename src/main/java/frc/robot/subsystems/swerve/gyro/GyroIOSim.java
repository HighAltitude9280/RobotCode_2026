package frc.robot.subsystems.swerve.gyro;

public class GyroIOSim implements GyroIO {
  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    // In simulation, we typically calculate gyro yaw based on swerve kinematics in
    // the Drive Subsystem
    // and set it here via a setter, or just let the pose estimator handle ideal
    // odometry.
    // For now, simple stub.
  }
}
