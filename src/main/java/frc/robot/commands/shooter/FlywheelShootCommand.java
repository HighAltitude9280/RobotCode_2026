package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HighAltitudeConstants;
import frc.robot.subsystems.shooter.Flywheel;

public class FlywheelShootCommand extends Command {
  private final Flywheel flywheel;

  public FlywheelShootCommand(Flywheel flywheel) {
    this.flywheel = flywheel;
    addRequirements(flywheel);
  }

  @Override
  public void execute() {
    flywheel.runVelocity(HighAltitudeConstants.Shooter.MAX_FLYWHEEL_SPEED_RADS_PER_SEC);
  }

  @Override
  public void end(boolean interrupted) {
    flywheel.coast();
  }
}
