package frc.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOInputsAutoLogged;
import frc.robot.util.PhoenixOdometryThread;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase {
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final SwerveModule[] modules;
  private final SwerveDrivePoseEstimator poseEstimator;

  // Cache de posiciones para no crear arrays en cada loop (Optimización)
  private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

  public SwerveDrive(GyroIO gyroIO, SwerveModule... modules) {
    this.gyroIO = gyroIO;
    this.modules = modules;

    // Inicializar array
    for (int i = 0; i < 4; i++) {
      modulePositions[i] = new SwerveModulePosition();
    }

    this.poseEstimator =
        new SwerveDrivePoseEstimator(
            HighAltitudeConstants.Swerve.KINEMATICS,
            new Rotation2d(),
            getModulePositions(),
            new Pose2d());
  }

  @Override
  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    for (SwerveModule module : modules) {
      module.periodic();
    }

    // --- ODOMETRÍA POWERHOUSE ---
    if (Robot.isReal()) {
      // 1. Hardware Real: Consumir la cola de Alta Frecuencia (250Hz)
      var queue = PhoenixOdometryThread.getInstance().getQueue();
      var sample = queue.poll(); // Extraer muestra

      while (sample != null) {
        // Convertir arrays de doubles a SwerveModulePosition[]
        for (int i = 0; i < 4; i++) {
          modulePositions[i].distanceMeters =
              sample.drivePositionsRad()[i] * HighAltitudeConstants.Swerve.WHEEL_RADIUS_METERS;
          modulePositions[i].angle = new Rotation2d(sample.turnPositionsRad()[i]);
        }

        // Actualizar Pose Estimator con timestamp preciso
        // (En un robot real usaríamos el gyro interpolado, aquí usamos el último
        // conocido por simplicidad
        // o idealmente tendríamos un thread de gyro también).
        poseEstimator.updateWithTime(
            sample.timestamp(), Rotation2d.fromRadians(gyroInputs.yawPositionRad), modulePositions);

        sample = queue.poll(); // Siguiente muestra
      }
    } else {
      // 2. Simulación: Discretización para reducir Drift
      var moduleStates = getModuleStates();
      var chassisSpeeds = HighAltitudeConstants.Swerve.KINEMATICS.toChassisSpeeds(moduleStates);

      // CORRECCIÓN DE SKEW (Discretize):
      // Predice dónde estará el robot al final del ciclo (20ms) considerando
      // rotación.
      var discretizedSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

      double yawIncrement = discretizedSpeeds.omegaRadiansPerSecond * 0.02;
      gyroInputs.yawPositionRad += yawIncrement; // Integración más precisa

      poseEstimator.update(Rotation2d.fromRadians(gyroInputs.yawPositionRad), getModulePositions());
    }

    Logger.recordOutput("Odometry/Robot", getPose());
    Logger.recordOutput("Odometry/ModuleStates", getModuleStates());
  }

  public void runVelocity(ChassisSpeeds speeds) {
    // CORRECCIÓN POWERHOUSE: Discretizar el setpoint también
    // Esto ayuda a que el robot siga arcos perfectos en lugar de polígonos.
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

    SwerveModuleState[] setpointStates =
        HighAltitudeConstants.Swerve.KINEMATICS.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, HighAltitudeConstants.Swerve.MAX_LINEAR_SPEED_M_S);

    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }
  }

  // ... (Resto de métodos: stop, zeroGyro, getPose... iguales)
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public void zeroGyro() {
    gyroIO.reset();
    Pose2d currentPose = getPose();
    resetPose(new Pose2d(currentPose.getTranslation(), new Rotation2d()));
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(
        Rotation2d.fromRadians(gyroInputs.yawPositionRad), getModulePositions(), pose);
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public SwerveModulePosition[] getModulePositions() {
    // Retorna las posiciones actuales (lectura directa para uso general)
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }
}
