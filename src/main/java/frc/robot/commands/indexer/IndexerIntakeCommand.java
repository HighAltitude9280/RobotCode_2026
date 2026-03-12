package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HighAltitudeConstants;
import frc.robot.subsystems.indexer.Indexer;

public class IndexerIntakeCommand extends Command {
  private final Indexer indexer;
  private boolean ejecting = false;
  private double ejectStartTime = 0;

  public IndexerIntakeCommand(Indexer indexer) {
    this.indexer = indexer;
    addRequirements(indexer);
  }

  @Override
  public void initialize() {
    ejecting = false;
  }

  @Override
  public void execute() {
    if (ejecting) {
      indexer.eject();
      // Stop ejecting after duration
      if (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - ejectStartTime 
          > HighAltitudeConstants.Indexer.EJECT_DURATION_SECONDS) {
        ejecting = false;
      }
    } else {
      if (indexer.isStalled()) {
        ejecting = true;
        ejectStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      } else {
        indexer.intake();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    indexer.stop();
  }
}