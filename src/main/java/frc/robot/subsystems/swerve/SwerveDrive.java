package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase {
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final SwerveModule[] modules;
  private final SwerveDriveKinematics kinematics;

  // TODO: Ajustar dimensiones reales en HighAltitudeConstants
  // Trackwidth: Distancia izquierda-derecha, Wheelbase: Distancia frente-atrás
  private final double TRACK_WIDTH_M = 0.5;
  private final double WHEEL_BASE_M = 0.5;

  public SwerveDrive(GyroIO gyroIO, SwerveModule... modules) {
    this.gyroIO = gyroIO;
    this.modules = modules;

    // FL, FR, BL, BR
    this.kinematics =
        new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE_M / 2.0, TRACK_WIDTH_M / 2.0),
            new Translation2d(WHEEL_BASE_M / 2.0, -TRACK_WIDTH_M / 2.0),
            new Translation2d(-WHEEL_BASE_M / 2.0, TRACK_WIDTH_M / 2.0),
            new Translation2d(-WHEEL_BASE_M / 2.0, -TRACK_WIDTH_M / 2.0));
  }

  @Override
  public void periodic() {
    // 1. Update Gyro
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    // 2. Update Modules
    for (SwerveModule module : modules) {
      module.periodic();
    }

    // 3. Log Pose / Odometry (Pendiente: añadir PoseEstimator aquí)
  }

  /**
   * * Método principal de control.
   *
   * @param speeds Velocidades en el chasis (m/s, rad/s)
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Convertir chassis speeds a module states
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);

    // Desaturar (si un módulo pide > max velocidad, escalar todos hacia abajo)
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, 4.5); // Max 4.5 m/s placeholder

    // Enviar a módulos
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }
  }

  public void stop() {
    runVelocity(new ChassisSpeeds());
  }
}
