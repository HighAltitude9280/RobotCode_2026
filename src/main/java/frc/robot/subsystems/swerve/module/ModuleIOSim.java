package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.HighAltitudeConstants;

public class ModuleIOSim implements ModuleIO {
  private final FlywheelSim driveSim;
  private final DCMotorSim turnSim;

  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim() {
    // --- DRIVE SIM (Kraken X60) ---
    var drivePlant =
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60(1), 0.025, HighAltitudeConstants.Swerve.DRIVE_GEAR_RATIO);

    driveSim =
        new FlywheelSim(
            drivePlant, DCMotor.getKrakenX60(1), HighAltitudeConstants.Swerve.DRIVE_GEAR_RATIO);

    // --- TURN SIM (NEO) ---
    var turnPlant =
        LinearSystemId.createDCMotorSystem(
            DCMotor.getNEO(1), 0.004, HighAltitudeConstants.Swerve.TURN_GEAR_RATIO);

    // CORRECCIÓN: Ponemos el ruido en 0.0 para evitar el drift en AdvantageScope
    turnSim =
        new DCMotorSim(
            turnPlant, DCMotor.getNEO(1), 0.0, 0.0 // <--- CERO RUIDO = Odometría Estable
            );
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    driveSim.setInput(driveAppliedVolts);
    turnSim.setInput(turnAppliedVolts);

    driveSim.update(0.02);
    turnSim.update(0.02);

    inputs.drivePositionRad += driveSim.getAngularVelocityRadPerSec() * 0.02;
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = driveSim.getCurrentDrawAmps();

    inputs.turnPositionRad = turnSim.getAngularPositionRad();
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = turnSim.getCurrentDrawAmps();

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
