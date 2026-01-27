package frc.robot.controls.profiles;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.HighAltitudeConstants;
import frc.robot.RobotContainer;
import frc.robot.controls.ControlProfile;

public class DefaultDriver implements ControlProfile {
  private final CommandXboxController controller;
  private final double DEADBAND = HighAltitudeConstants.Controls.DEADBAND;

  public DefaultDriver() {
    // Puerto 0 por defecto para el Driver
    this.controller = new CommandXboxController(0);
  }

  @Override
  public double getDriveForward() {
    // Joystick Y negativo es "arriba". Invertimos para que + sea adelante.
    double raw = -controller.getLeftY();
    return processAxis(raw);
  }

  @Override
  public double getDriveStrafe() {
    // Joystick X positivo es "derecha". Invertimos para que + sea izquierda (NWU).
    double raw = -controller.getLeftX();
    return processAxis(raw);
  }

  @Override
  public double getDriveRotation() {
    // Joystick X positivo es "derecha". Invertimos para que + sea CCW.
    double raw = -controller.getRightX();
    return processAxis(raw);
  }

  @Override
  public void configureBindings(RobotContainer container) {
    // Botón Back para resetear el giroscopio (Field Centric Reset)
    controller.back().onTrue(container.getDrive().runOnce(() -> container.getDrive().zeroGyro()));
  }

  /** Aplica Deadband y Squaring para mayor precisión. */
  private double processAxis(double value) {
    // 1. Aplicar Deadband
    value = MathUtil.applyDeadband(value, DEADBAND);

    // 2. Elevar al cuadrado manteniendo el signo (Squaring)
    // Esto da más control fino a bajas velocidades.
    return Math.copySign(value * value, value);
  }
}
