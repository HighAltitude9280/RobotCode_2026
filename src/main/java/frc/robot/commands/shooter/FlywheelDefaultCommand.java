package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HighAltitudeConstants;
import frc.robot.controls.ControlProfile;
import frc.robot.subsystems.Shooter.Flywheel;

public class FlywheelDefaultCommand extends Command {
  private final Flywheel flywheel;
  private final ControlProfile controller;

  public FlywheelDefaultCommand(Flywheel flywheel, ControlProfile controller) {
    this.flywheel = flywheel;
    this.controller = controller;
    addRequirements(flywheel);
  }

  @Override
  public void execute() {
    // 1. Get input
    double triggerInput = controller.getShooterTrigger();

    // 2. Scale to rad/s
    double targetVelocity =
        triggerInput * HighAltitudeConstants.Shooter.MAX_FLYWHEEL_SPEED_RADS_PER_SEC;

    // 3. Send to subsystem
    if (triggerInput > HighAltitudeConstants.Controls.DEADBAND) {
      flywheel.runVelocity(targetVelocity);
    } else {
      flywheel.coast();
    }
  }

  @Override
  public void end(boolean interrupted) {
    flywheel.coast();
  }
}
