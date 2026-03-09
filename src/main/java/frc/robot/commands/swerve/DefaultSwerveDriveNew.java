package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HighAltitudeConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DefaultSwerveDriveNew extends Command {
  private final SwerveDrive swerve;
  private final DoubleSupplier vXSupplier;
  private final DoubleSupplier vYSupplier;
  private final DoubleSupplier omegaSupplier;
  private final BooleanSupplier precisionModeSupplier;
  private final BooleanSupplier fieldOrientedSupplier;

  /**
   * Comando por defecto para el Swerve Drive optimizado para 2026.
   * Implementa curva exponencial agresiva para los Krakens, compensación de Skew
   * mediante discretización y compatibilidad estricta con AdvantageKit
   * (Replay-safe).
   */
  public DefaultSwerveDriveNew(
      SwerveDrive swerve,
      DoubleSupplier vXSupplier,
      DoubleSupplier vYSupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier precisionModeSupplier,
      BooleanSupplier fieldOrientedSupplier) {
    this.swerve = swerve;
    this.vXSupplier = vXSupplier;
    this.vYSupplier = vYSupplier;
    this.omegaSupplier = omegaSupplier;
    this.precisionModeSupplier = precisionModeSupplier;
    this.fieldOrientedSupplier = fieldOrientedSupplier;

    addRequirements(swerve);
  }

  @Override
  public void execute() {
    // 1. Leer valores de los Suppliers (Replay-safe)
    double vX = vXSupplier.getAsDouble();
    double vY = vYSupplier.getAsDouble();
    double omega = omegaSupplier.getAsDouble();
    boolean isPrecisionMode = precisionModeSupplier.getAsBoolean();
    boolean isFieldOriented = fieldOrientedSupplier.getAsBoolean();

    // 2. Deadband Centralizado = DEADZONE
    vX = MathUtil.applyDeadband(vX, 0.1);
    vY = MathUtil.applyDeadband(vY, 0.1);
    omega = MathUtil.applyDeadband(omega, 0.1);

    // 3. Curva Exponencial de Aceleración (Squaring preservando el signo)
    vX = Math.copySign(vX * vX, vX);
    vY = Math.copySign(vY * vY, vY);
    omega = Math.copySign(omega * omega, omega);

    // 4. Multiplicadores de Modo (Precisión vs Sprints) = PRECISION MODE
    double transMultiplier = isPrecisionMode ? 0.30 : 0.75;
    double rotMultiplier = isPrecisionMode ? 0.50 : 0.65;

    // Aplicar multiplicadores y escalar a velocidades físicas máximas
    vX *= transMultiplier * HighAltitudeConstants.Swerve.MAX_LINEAR_SPEED_M_S;
    vY *= transMultiplier * HighAltitudeConstants.Swerve.MAX_LINEAR_SPEED_M_S;
    omega *= rotMultiplier * HighAltitudeConstants.Swerve.MAX_ANGULAR_SPEED_RAD_S;

    // 5. Transformación Field-Oriented vs Robot-Oriented
    ChassisSpeeds desiredSpeeds;
    if (isFieldOriented) {
      desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          vX, vY, omega, swerve.getRotation());
    } else {
      desiredSpeeds = new ChassisSpeeds(vX, vY, omega);
    }

    // 6. Skew Compensation (Discretización a 50Hz)
    // Esto inyecta micro-compensaciones en giros agresivos para no perder tracción
    ChassisSpeeds discretSpeeds = ChassisSpeeds.discretize(desiredSpeeds, 0.02);

    // 7. Enviar comandos al hardware genérico
    swerve.runVelocity(discretSpeeds);

    // 8. AdvantageKit Logging de telemetría procesada
    Logger.recordOutput("Swerve/Commands/DefaultDrive/IsPrecisionMode", isPrecisionMode);
    Logger.recordOutput("Swerve/Commands/DefaultDrive/IsFieldOriented", isFieldOriented);
    Logger.recordOutput("Swerve/Commands/DefaultDrive/vxMetersPerSecond", discretSpeeds.vxMetersPerSecond);
    Logger.recordOutput("Swerve/Commands/DefaultDrive/vyMetersPerSecond", discretSpeeds.vyMetersPerSecond);
    Logger.recordOutput("Swerve/Commands/DefaultDrive/omegaRadiansPerSecond", discretSpeeds.omegaRadiansPerSecond);
  }

  @Override
  public void end(boolean interrupted) {
    // Enviar ceros absolutos al interrumpirse o cancelar para frenar de inmediato
    swerve.runVelocity(new ChassisSpeeds());
  }
}