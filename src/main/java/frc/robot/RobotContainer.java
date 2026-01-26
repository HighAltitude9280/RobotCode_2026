package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.gyro.GyroIONavX;
import frc.robot.subsystems.swerve.gyro.GyroIOSim;
import frc.robot.subsystems.swerve.module.ModuleIOSim;
import frc.robot.subsystems.swerve.module.ModuleIOTalonSpark;

public class RobotContainer {
  private final SwerveDrive drive;

  public RobotContainer() {
    if (Robot.isReal()) {
      // Real Hardware Injection
      // TODO: Reemplazar los números mágicos (IDs) con HighAltitudeConstants
      // Orden: driveID, turnID, cancoderID, driveGearRatio, turnGearRatio,
      // driveInverted
      drive =
          new SwerveDrive(
              new GyroIONavX(),
              new SwerveModule(new ModuleIOTalonSpark(1, 2, 9, 6.75, 12.8, false), 0), // FL
              new SwerveModule(new ModuleIOTalonSpark(3, 4, 10, 6.75, 12.8, false), 1), // FR
              new SwerveModule(new ModuleIOTalonSpark(5, 6, 11, 6.75, 12.8, false), 2), // BL
              new SwerveModule(new ModuleIOTalonSpark(7, 8, 12, 6.75, 12.8, false), 3) // BR
              );
    } else {
      // Sim Injection
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

  private void configureBindings() {
    // Default Command simple para probar que no crashea
    drive.setDefaultCommand(new RunCommand(() -> drive.stop(), drive));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No Auto Configured");
  }
}
