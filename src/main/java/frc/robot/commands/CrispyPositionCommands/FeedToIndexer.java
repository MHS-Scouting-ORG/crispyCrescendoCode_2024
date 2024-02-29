
package frc.robot.commands.CrispyPositionCommands;

//NO BIEN 
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShindexerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.UnderIntakeSubsystem;

public class FeedToIndexer extends Command {
  IndexerSubsystem indexSub;
  UnderIntakeSubsystem underIntakeSub;
  public FeedToIndexer(IndexerSubsystem indexSub, UnderIntakeSubsystem underIntakeSub) {
    this.indexSub = indexSub;
    this.underIntakeSub = underIntakeSub; 
    addRequirements(indexSub);
    addRequirements(underIntakeSub);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    underIntakeSub.intake(IntakeConstants.INTAKE_MAXSPEED);
    indexSub.index(ShindexerConstants.INDEXER_SPEED);
  }

  public void end(boolean interrupted) {
    underIntakeSub.stopIntake();
    indexSub.stop();
  }

  @Override
  public boolean isFinished() {
    return indexSub.getOpticalSwitch();
  }
}
