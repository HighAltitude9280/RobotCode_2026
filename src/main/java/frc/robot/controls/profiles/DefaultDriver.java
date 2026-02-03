package frc.robot.controls.profiles;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants;
import frc.robot.HighAltitudeConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.swerve.DriveToPose;
import frc.robot.commands.swerve.SwerveHeadingLockCommand;
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

    // --- DRIVER ASSISTS ---

    controller
        .x()
        .whileTrue(
            new DriveToPose(
                container.getDrive(),
                () -> FieldConstants.onAlliance(FieldConstants.BLUE_SHOOTING_LEFT),
                false));

    // Botón Y: Snap to HUB (Centro)
    // Permite orbitar el Hub disparando.
    controller
        .y()
        .whileTrue(
            new SwerveHeadingLockCommand(
                container.getDrive(),
                () -> -controller.getLeftY(), // Forward (revisa tus inverts)
                () -> -controller.getLeftX(), // Strafe
                () -> FieldConstants.onAlliance(FieldConstants.BLUE_HUB_CENTER) // Target Dinámico
                ));

    // Botón A: Snap to FEEDER / ALLIANCE WALL (180 Grados)
    // Apunta hacia atrás para recoger game pieces o defender zona.
    controller
        .a()
        .whileTrue(
            new SwerveHeadingLockCommand(
                container.getDrive(),
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> FieldConstants.onAlliance(FieldConstants.BLUE_ALLIANCE_WALL_TARGET)));

    // Reset Gyro (Start Button)
    controller.start().onTrue(container.getDrive().runOnce(() -> container.getDrive().zeroGyro()));
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
