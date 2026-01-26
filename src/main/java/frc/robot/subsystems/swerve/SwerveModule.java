package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.module.ModuleIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  // TODO: Mover ganancias a HighAltitudeConstants
  private final PIDController turnPID = new PIDController(7.0, 0.0, 0.0);
  private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.1, 0.13);

  public SwerveModule(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
    turnPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + index, inputs);
  }

  /** Comando principal: Mover el módulo al estado deseado */
  public void runSetpoint(SwerveModuleState state) {
    // Optimizar giro (evitar girar más de 90 grados si se puede invertir la rueda)
    SwerveModuleState optimizedState =
        SwerveModuleState.optimize(state, new Rotation2d(inputs.turnPositionRad));

    // Turn Control (PID en RIO)
    double turnVolts = turnPID.calculate(inputs.turnPositionRad, optimizedState.angle.getRadians());
    io.setTurnVoltage(turnVolts);

    // Drive Control (Feedforward simple por ahora)
    double driveVolts = driveFF.calculate(optimizedState.speedMetersPerSecond);
    io.setDriveVoltage(driveVolts);
  }

  /** Para Odometría */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        inputs.drivePositionRad, new Rotation2d(inputs.turnPositionRad));
  }

  /** Stop (útil para tests) */
  public void stop() {
    io.setDriveVoltage(0);
    io.setTurnVoltage(0);
  }
}
