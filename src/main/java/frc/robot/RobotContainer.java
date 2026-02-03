package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.HighAltitudeConstants.Swerve;
import frc.robot.HighAltitudeConstants.Swerve.ModuleConstants;
import frc.robot.commands.swerve.SwerveDefaultCommand;
import frc.robot.controls.profiles.DefaultDriver;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.gyro.GyroIONavX;
import frc.robot.subsystems.swerve.gyro.GyroIOSim;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.module.ModuleIOSim;
import frc.robot.subsystems.swerve.module.ModuleIOTalonSpark;

public class RobotContainer {
  private final SwerveDrive drive;

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
    } else {
      // Simulation
      drive =
          new SwerveDrive(
              new GyroIOSim(),
              new SwerveModule(new ModuleIOSim(), 0),
              new SwerveModule(new ModuleIOSim(), 1),
              new SwerveModule(new ModuleIOSim(), 2),
              new SwerveModule(new ModuleIOSim(), 3));
    }

    configureBindings();
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
    drive.setDefaultCommand(new SwerveDefaultCommand(drive, new DefaultDriver()));
    new DefaultDriver().configureBindings(this);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No Auto Configured");
  }

  public SwerveDrive getDrive() {
    return drive;
  }
}
