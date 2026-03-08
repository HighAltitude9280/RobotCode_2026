package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.HighAltitudeConstants.Swerve;
import frc.robot.HighAltitudeConstants.Swerve.ModuleConstants;
import frc.robot.commands.swerve.SwerveDefaultCommand;
import frc.robot.controls.profiles.DefaultDriver;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.pivot.Pivot;
import frc.robot.subsystems.intake.pivot.PivotIO;
import frc.robot.subsystems.intake.pivot.PivotIOSparkMax;
import frc.robot.subsystems.rollers.RollerIO;
import frc.robot.subsystems.rollers.RollerIOTalonFX;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.FlywheelIO;
import frc.robot.subsystems.shooter.FlywheelIOTalonFX;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.gyro.GyroIONavX;
import frc.robot.subsystems.swerve.gyro.GyroIOSim;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.module.ModuleIOSim;
import frc.robot.subsystems.swerve.module.ModuleIOTalonSpark;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final SwerveDrive drive;
  private final Flywheel flywheel;
  private final Pivot intakePivot;
  private final Indexer indexer;
  private final Intake intake;
  private final Feeder feeder;

  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    if (Robot.isReal()) {
      // Real Hardware
      drive =
          new SwerveDrive(
              new GyroIONavX(),
              createRealModule(Swerve.MOD_FL, 0),
              createRealModule(Swerve.MOD_FR, 1),
              createRealModule(Swerve.MOD_BL, 2),
              createRealModule(Swerve.MOD_BR, 3));

      flywheel =
          new Flywheel(
              new FlywheelIOTalonFX(
                  HighAltitudeConstants.Shooter.ShooterRight,
                  HighAltitudeConstants.Shooter.ShooterLeft));

      intakePivot = new Pivot(new PivotIOSparkMax(HighAltitudeConstants.Pivot.PIVOTMOTOR));

      indexer = new Indexer(new RollerIOTalonFX(HighAltitudeConstants.Indexer.INDEXERMOTOR));

      intake = new Intake(new RollerIOTalonFX(HighAltitudeConstants.Intake.INTAKEMOTOR));

      feeder = new Feeder(new RollerIOTalonFX(HighAltitudeConstants.Feeder.FEEDERMOTOR));
    } else {
      // Simulation
      drive =
          new SwerveDrive(
              new GyroIOSim(),
              new SwerveModule(new ModuleIOSim(), 0),
              new SwerveModule(new ModuleIOSim(), 1),
              new SwerveModule(new ModuleIOSim(), 2),
              new SwerveModule(new ModuleIOSim(), 3));

      flywheel = new Flywheel(new FlywheelIO() {});
      intakePivot = new Pivot(new PivotIO() {});
      indexer = new Indexer(new RollerIO() {});
      intake = new Intake(new RollerIO() {});
      feeder = new Feeder(new RollerIO() {});
    }

    configureBindings();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    setupAutonomousCommands();
  }

  private SwerveModule createRealModule(ModuleConstants constants, int index) {
    ModuleIO io =
        new ModuleIOTalonSpark(
            constants.driveID(), // CORREGIDO: antes driveMotorID()
            constants.turnID(), // CORREGIDO: antes turnMotorID()
            constants.cancoderID(),
            constants.offset(), // AGREGADO: Pasamos el offset del encoder
            Swerve.DRIVE_GEAR_RATIO,
            Swerve.TURN_GEAR_RATIO,
            constants.driveInverted());
    return new SwerveModule(io, index);
  }

  private void configureBindings() {
    DefaultDriver driver = new DefaultDriver();
    drive.setDefaultCommand(new SwerveDefaultCommand(drive, driver));
    flywheel.setDefaultCommand(Commands.run(() -> flywheel.coast(), flywheel));
    driver.configureBindings(this);
  }

  /** Configures all the auto routines available on the Dashboard. */
  private void setupAutonomousCommands() {
    // 🛑 REGLA DE ORO POWERHOUSE: El default SIEMPRE es "Do Nothing".
    // Si el Driver olvida seleccionar un auto, el robot se queda quieto.
    // Es mejor perder 15 puntos que estrellarse contra la pared a 5 m/s.
    autoChooser.addDefaultOption("Do Nothing", Commands.none());

    // 🟢 Cargar Autos de PathPlanner
    autoChooser.addOption("AL1 (Auto Left 1)", new PathPlannerAuto("AutoLeft1"));

    // Ejemplo de cómo agregar más en el futuro:
    // autoChooser.addOption("AR1 (Auto Right 1)", new PathPlannerAuto("AR1"));
    // autoChooser.addOption("Test 2 Meters", new PathPlannerAuto("Test2M"));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public SwerveDrive getDrive() {
    return drive;
  }

  public Flywheel getFlywheel() {
    return flywheel;
  }

  public Pivot getPivot() {
    return intakePivot;
  }

  public Indexer getIndexer() {
    return indexer;
  }

  public Intake getIntake() {
    return intake;
  }

  public Feeder getFeeder() {
    return feeder;
  }
}
