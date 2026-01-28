package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.HighAltitudeConstants;

public class ModuleIOSim implements ModuleIO {

  // Simuladores Físicos
  private final FlywheelSim driveSim;
  private final DCMotorSim turnSim;

  // Estado de Voltajes
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim() {
    System.out.println("[Init] Creating ModuleIOSim PowerHouse Style");

    // --- DRIVE SIMULATION (Kraken X60) ---
    // 1. Crear Planta (Modelo Físico)
    var drivePlant =
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60(1),
            HighAltitudeConstants.MOI_DRIVE_KG_M2,
            HighAltitudeConstants.Swerve.DRIVE_GEAR_RATIO);

    // 2. Crear Simulador
    driveSim =
        new FlywheelSim(
            drivePlant, DCMotor.getKrakenX60(1), 0.0 // <--- Ruido de Velocidad (double... varargs)
            );

    // --- TURN SIMULATION (NEO) ---
    // 1. Crear Planta
    var turnPlant =
        LinearSystemId.createDCMotorSystem(
            DCMotor.getNEO(1),
            HighAltitudeConstants.MOI_TURN_KG_M2,
            HighAltitudeConstants.Swerve.TURN_GEAR_RATIO);

    // 2. Crear Simulador
    turnSim =
        new DCMotorSim(
            turnPlant, DCMotor.getNEO(1), 0.0, 0.0 // <--- Ruido de [Pos, Vel] (double... varargs)
            );
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // 1. Aplicar Voltajes
    driveSim.setInputVoltage(driveAppliedVolts);
    turnSim.setInputVoltage(turnAppliedVolts);

    // 2. Avanzar Física (20ms)
    driveSim.update(0.02);
    turnSim.update(0.02);

    // 3. Output a Inputs

    // Drive (FlywheelSim es velocidad -> Integramos posición manualmente)
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.drivePositionRad += inputs.driveVelocityRadPerSec * 0.02;
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = driveSim.getCurrentDrawAmps();

    // Turn (DCMotorSim ya maneja posición)
    inputs.turnPositionRad = turnSim.getAngularPositionRad();
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = turnSim.getCurrentDrawAmps();

    // Encoder Absoluto (Perfecto en Sim)
    inputs.turnAbsolutePositionRad = inputs.turnPositionRad;
  }

  @Override
  public void setDriveVoltage(double volts) {
    this.driveAppliedVolts = volts;
  }

  @Override
  public void setTurnVoltage(double volts) {
    this.turnAppliedVolts = volts;
  }
}
