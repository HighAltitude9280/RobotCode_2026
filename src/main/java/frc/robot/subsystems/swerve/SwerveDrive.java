package frc.robot.subsystems.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

  // Cache de posiciones para no crear arrays en cada loop (Optimización de
  // Memoria)
  private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

  public SwerveDrive(GyroIO gyroIO, SwerveModule... modules) {
    this.gyroIO = gyroIO;
    this.modules = modules;

    // Inicializar array de cache
    for (int i = 0; i < 4; i++) {
      modulePositions[i] = new SwerveModulePosition();
    }

    this.poseEstimator =
        new SwerveDrivePoseEstimator(
            HighAltitudeConstants.Swerve.KINEMATICS,
            new Rotation2d(),
            getModulePositions(), // Usa el método optimizado
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
      var sample = queue.poll();

      while (sample != null) {
        // Actualizamos el cache directamente
        for (int i = 0; i < 4; i++) {
          modulePositions[i].distanceMeters =
              sample.drivePositionsRad()[i] * HighAltitudeConstants.Swerve.WHEEL_RADIUS_METERS;
          modulePositions[i].angle = new Rotation2d(sample.turnPositionsRad()[i]);
        }

        // Actualizar Pose Estimator con timestamp preciso
        poseEstimator.updateWithTime(
            sample.timestamp(),
            Rotation2d.fromRadians(gyroInputs.yawPositionRad),
            modulePositions // Pasamos el array cacheado
            );

        sample = queue.poll();
      }
    } else {
      // 2. Simulación: Discretización para reducir Drift
      var moduleStates = getModuleStates();
      var chassisSpeeds = HighAltitudeConstants.Swerve.KINEMATICS.toChassisSpeeds(moduleStates);

      // CORRECCIÓN DE SKEW (Discretize)
      var discretizedSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

      double yawIncrement = discretizedSpeeds.omegaRadiansPerSecond * 0.02;
      gyroInputs.yawPositionRad += yawIncrement;

      poseEstimator.update(Rotation2d.fromRadians(gyroInputs.yawPositionRad), getModulePositions());
    }

    Logger.recordOutput("Odometry/Robot", getPose());
    Logger.recordOutput("Odometry/ModuleStates", getModuleStates());
  }

  /** Control principal de velocidad. */
  public void runVelocity(ChassisSpeeds speeds) {
    // CORRECCIÓN POWERHOUSE: Discretizar el setpoint
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

    SwerveModuleState[] setpointStates =
        HighAltitudeConstants.Swerve.KINEMATICS.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, HighAltitudeConstants.Swerve.MAX_LINEAR_SPEED_M_S);

    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }
  }

  /**
   * [NUEVO] Control de velocidad con centro de rotación variable. Útil para rotar alrededor de una
   * esquina del robot o un game piece.
   */
  public void runVelocity(ChassisSpeeds speeds, Translation2d centerOfRotation) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

    SwerveModuleState[] setpointStates =
        HighAltitudeConstants.Swerve.KINEMATICS.toSwerveModuleStates(
            discreteSpeeds, centerOfRotation);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, HighAltitudeConstants.Swerve.MAX_LINEAR_SPEED_M_S);

    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }
  }

  /**
   * [NUEVO] Retorna la velocidad del robot RELATIVA AL CAMPO. Requerido para DriveToPose y
   * PathPlanner.
   */
  public ChassisSpeeds getFieldRelativeSpeeds() {
    // 1. Obtener velocidades relativas al robot
    ChassisSpeeds robotSpeeds =
        HighAltitudeConstants.Swerve.KINEMATICS.toChassisSpeeds(getModuleStates());
    // 2. Rotar por el ángulo actual para obtener velocidades de campo
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, getRotation());
  }

  /**
   * [NUEVO] Pone los módulos en X (X-Stance) para defensa. Hace muy difícil que empujen el robot.
   */
  public void lock() {
    SwerveModuleState[] states =
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)), // FL
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), // FR
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), // BL
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)) // BR
        };
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(states[i]);
    }
  }

  /**
   * [NUEVO] Método para inyectar datos de Visión (PhotonVision/Limelight). Los equipos PowerHouse
   * mezclan odometría + visión aquí.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp, stdDevs);
  }

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

  /** [OPTIMIZADO] Retorna posiciones reusando el array de memoria. */
  public SwerveModulePosition[] getModulePositions() {
    for (int i = 0; i < 4; i++) {
      // Actualizamos el objeto existente en lugar de crear uno nuevo
      // OJO: modules[i].getPosition() en SwerveModule también debe ser eficiente.
      // Si SwerveModule crea un nuevo objeto cada vez, aquí solo copiamos la
      // referencia.
      // Lo ideal es copiar valores, pero por ahora esto evita crear el array
      // contenedor.
      modulePositions[i] = modules[i].getPosition();
    }
    return modulePositions;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }
}
