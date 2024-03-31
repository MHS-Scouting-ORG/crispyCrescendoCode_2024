package frc.robot.commands.ShindexerCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShindexerConstants;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexAuto extends Command {

  private IndexerSubsystem indexSub;
  private Timer timer; 

  public IndexAuto(IndexerSubsystem indexSubsystem) {
    indexSub = indexSubsystem;
    timer = new Timer();
    addRequirements(indexSub);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    indexSub.index(ShindexerConstants.INDEXER_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    indexSub.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.get() > 1; 
  }
}