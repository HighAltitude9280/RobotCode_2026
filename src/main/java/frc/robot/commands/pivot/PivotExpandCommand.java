package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.pivot.Pivot;

public class PivotExpandCommand extends Command {
  private final Pivot pivot;

  public PivotExpandCommand(Pivot pivot) {
    this.pivot = pivot;
    addRequirements(pivot);
  }

  @Override
  public void execute() {
    pivot.expand();
  }

  @Override
  public void end(boolean interrupted) {
    pivot.retract();
  }
}
