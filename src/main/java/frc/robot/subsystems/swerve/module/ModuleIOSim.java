package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ModuleIOSim implements ModuleIO {
  // Sim Objects
  // CORRECCIÓN: Usar LinearSystemId para definir el modelo físico explícitamente
  private final FlywheelSim driveSim;
  private final DCMotorSim turnSim;

  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim() {
    // Drive: Kraken X60, Gearing 6.75, Inertia 0.025 (kg*m^2)
    // LinearSystemId.createFlywheelSystem(motor, J, gearing)
    driveSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.025, 6.75),
            DCMotor.getKrakenX60(1),
            6.75);

    // Turn: NEO, Gearing 12.8, Inertia 0.004
    // LinearSystemId.createDCMotorSystem(motor, J, gearing)
    turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 12.8),
            DCMotor.getNEO(1),
            12.8);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update Physics (loop time 20ms standard)
    driveSim.setInput(driveAppliedVolts);
    turnSim.setInput(turnAppliedVolts);

    driveSim.update(0.02);
    turnSim.update(0.02);

    // Write to Inputs
    inputs.drivePositionRad += driveSim.getAngularVelocityRadPerSec() * 0.02;
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = driveSim.getCurrentDrawAmps();

    inputs.turnPositionRad = turnSim.getAngularPositionRad();
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = turnSim.getCurrentDrawAmps();

    // Simular encoder absoluto perfecto
    inputs.turnAbsolutePositionRad = inputs.turnPositionRad;
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveAppliedVolts = volts;
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnAppliedVolts = volts;
  }
}
