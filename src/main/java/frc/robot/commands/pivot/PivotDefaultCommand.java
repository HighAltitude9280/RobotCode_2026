package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.pivot.Pivot;

public class PivotDefaultCommand extends Command {
  private final Pivot pivot;

  public PivotDefaultCommand(Pivot pivot) {
    this.pivot = pivot;
    addRequirements(pivot);
  }

  @Override
  public void initialize() {
    pivot.retract();
  }

  @Override
  public void execute() {
    pivot.retract();
  }
}
