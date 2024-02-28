package frc.robot.commands.ShindexerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShindexerConstants;
import frc.robot.subsystems.IndexerSubsystem;

public class IntakeToIndexerCommand extends Command {
  
  private IndexerSubsystem index_Sub;

  public IntakeToIndexerCommand(IndexerSubsystem indexer_Sub) {
    index_Sub = indexer_Sub;
    addRequirements(index_Sub);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    index_Sub.index(ShindexerConstants.INDEXER_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    while(index_Sub.getOpticalSwitch()){
      index_Sub.index(ShindexerConstants.INDEXER_SPEED);
    }
    index_Sub.stop();
  }

  @Override
  public boolean isFinished() {
    return index_Sub.getOpticalSwitch();
  }
}
