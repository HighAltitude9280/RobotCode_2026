package frc.robot.controls;

import frc.robot.RobotContainer;

public interface ControlProfile {
  /** Retorna la velocidad frontal deseada (-1.0 a 1.0). Positivo = Hacia adelante. */
  double getDriveForward();

  /**
   * Retorna la velocidad lateral deseada (-1.0 a 1.0). Positivo = Hacia la izquierda (Estándar
   * WPILib NWU).
   */
  double getDriveStrafe();

  /**
   * Retorna la velocidad de rotación deseada (-1.0 a 1.0). Positivo = Sentido Antihorario (CCW).
   */
  double getDriveRotation();

  /**
   * Configura los botones específicos de este perfil.
   *
   * @param container Referencia al contenedor para acceder a los subsistemas.
   */
  void configureBindings(RobotContainer container);
}
