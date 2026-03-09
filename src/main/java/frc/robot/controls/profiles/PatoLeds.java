package frc.robot.controls.profiles;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.HighAltitudeConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.leds.LedsBlueCommand;
import frc.robot.commands.leds.LedsGreenCommand;
import frc.robot.commands.leds.LedsHARCommand;
import frc.robot.controls.ControlProfile;

public class PatoLeds implements ControlProfile {
  private final CommandXboxController controller;
  private final double DEADBAND = HighAltitudeConstants.Controls.DEADBAND;

  public PatoLeds() {
    // Puerto 1 por defecto para el Pato
    this.controller = new CommandXboxController(1);
  }

  @Override
  public double getDriveForward() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getDriveForward'");
  }

  @Override
  public double getDriveStrafe() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getDriveStrafe'");
  }

  @Override
  public double getDriveRotation() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getDriveRotation'");
  }

  @Override
  public void configureBindings(RobotContainer container) {
    controller.a().whileTrue(new LedsGreenCommand(container.getLeds()));
    controller.b().whileTrue(new LedsBlueCommand(container.getLeds()));
    controller.x().onTrue(new LedsHARCommand(container.getLeds()));
  }
}
