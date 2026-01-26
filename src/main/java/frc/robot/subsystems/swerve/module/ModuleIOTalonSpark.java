package frc.robot.subsystems.swerve.module;

import static edu.wpi.first.units.Units.*; // Para conversiones (Volts, Radians, etc.)

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.*; // Importante para tipos Angle, Voltage, etc.

public class ModuleIOTalonSpark implements ModuleIO {
  // Hardware
  private final TalonFX driveTalon;
  private final SparkMax turnSpark;
  private final CANcoder turnCanCoder;
  private final RelativeEncoder turnRelativeEncoder;

  // Status Signals (Phoenix 6 Typed Signals)
  // CORRECCIÓN: Tipos estrictos en lugar de <Double>
  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;
  private final StatusSignal<Temperature> driveTemp;
  private final StatusSignal<Angle> turnAbsolutePos;

  // Control Requests
  private final VoltageOut driveVoltageRequest = new VoltageOut(0);

  // Constants
  private final double DRIVE_GEAR_RATIO;
  private final double TURN_GEAR_RATIO;

  public ModuleIOTalonSpark(
      int driveID,
      int turnID,
      int cancoderID,
      double driveGearRatio,
      double turnGearRatio,
      boolean driveInverted) {
    this.DRIVE_GEAR_RATIO = driveGearRatio;
    this.TURN_GEAR_RATIO = turnGearRatio;

    // 1. Configure Drive (TalonFX - Phoenix 6)
    driveTalon = new TalonFX(driveID, "rio");
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.Inverted =
        driveInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    driveTalon.getConfigurator().apply(driveConfig);

    // 2. Configure Turn (SparkMax - REVLib 2026)
    turnSpark = new SparkMax(turnID, MotorType.kBrushless);
    turnRelativeEncoder = turnSpark.getEncoder();

    SparkMaxConfig turnConfig = new SparkMaxConfig();
    turnConfig.idleMode(IdleMode.kBrake);
    turnConfig.smartCurrentLimit(30);
    turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // 3. Configure CANCoder (Phoenix 6)
    turnCanCoder = new CANcoder(cancoderID, "rio");
    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    turnCanCoder.getConfigurator().apply(cancoderConfig);

    // 4. Signal Setup for Logging
    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getSupplyCurrent();
    driveTemp = driveTalon.getDeviceTemp();
    turnAbsolutePos = turnCanCoder.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        driveTemp,
        turnAbsolutePos);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Refresh Phoenix 6 Signals
    BaseStatusSignal.refreshAll(
        drivePosition, driveVelocity, driveAppliedVolts, driveCurrent, driveTemp, turnAbsolutePos);

    // CORRECCIÓN: Convertir explícitamente usando .in(Units)
    inputs.drivePositionRad = drivePosition.getValue().in(Radians) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        driveVelocity.getValue().in(RadiansPerSecond) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValue().in(Volts);
    inputs.driveCurrentAmps = driveCurrent.getValue().in(Amps);
    inputs.driveTempCelcius = driveTemp.getValue().in(Celsius);

    // Turn (SparkMax - REVLib units are usually Rotations by default, check your specific config)
    // Asumiendo default REV: Pos=Rotations, Vel=RPM
    inputs.turnPositionRad =
        edu.wpi.first.math.util.Units.rotationsToRadians(turnRelativeEncoder.getPosition())
            / TURN_GEAR_RATIO;
    inputs.turnVelocityRadPerSec =
        edu.wpi.first.math.util.Units.rotationsPerMinuteToRadiansPerSecond(
                turnRelativeEncoder.getVelocity())
            / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnSpark.getAppliedOutput() * turnSpark.getBusVoltage();
    inputs.turnCurrentAmps = turnSpark.getOutputCurrent();

    // Absolute (Phoenix 6 units: Rotations -> Radians via Units lib is safer, but Phoenix returns
    // rotations by default for Angle type??)
    // Phoenix 6 Angle type default unit is Rotations.
    inputs.turnAbsolutePositionRad =
        drivePosition.getValue().in(Rotations)
            * 2.0
            * Math.PI; // O usar .in(Radians) si Phoenix lo soporta directo
    // Mejor práctica Phoenix 6 + WPILib Units:
    inputs.turnAbsolutePositionRad = turnAbsolutePos.getValue().in(Radians);
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(driveVoltageRequest.withOutput(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSpark.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    driveTalon.getConfigurator().refresh(config);
    config.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    turnSpark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
