package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerCommand extends Command {

  private IndexerSubsystem indexSub;

  public IndexerCommand(IndexerSubsystem indexSubsystem) {
    indexSub = indexSubsystem;
    addRequirements(indexSub);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    indexSub.index();
  }

  @Override
  public void end(boolean interrupted) {
    indexSub.resetIndexerEnc();
    indexSub.stop();
  }

  @Override
  public boolean isFinished() {
    return indexSub.getOpticalSwitch();
  }
}