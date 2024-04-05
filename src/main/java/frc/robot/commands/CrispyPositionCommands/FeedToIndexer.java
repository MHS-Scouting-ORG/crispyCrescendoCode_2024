
package frc.robot.commands.CrispyPositionCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//NO BIEN 
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShindexerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.UnderIntakeSubsystem;

public class FeedToIndexer extends Command {
  IndexerSubsystem indexSub;
  UnderIntakeSubsystem underIntakeSub;
  private boolean noteSensed = false; 

  public FeedToIndexer(IndexerSubsystem indexSub, UnderIntakeSubsystem underIntakeSub) {
    this.indexSub = indexSub;
    this.underIntakeSub = underIntakeSub; 
    addRequirements(indexSub);
    addRequirements(underIntakeSub);
  }

  @Override
  public void initialize() {
    noteSensed = false; 
  }

  @Override
  public void execute() {
    // if (underIntakeSub.getOpticalSensor()) {
    //   noteSensed = true; 
    // }

    // if (noteSensed) {
      SmartDashboard.putString("CURRENT CMD", getName());
      underIntakeSub.intake(0.4);
      indexSub.index(ShindexerConstants.INDEXER_SPEED);
    // }
  }

  public void end(boolean interrupted) {
    SmartDashboard.putString("CURRENT CMD", "NONE");
    underIntakeSub.stopIntake();
    indexSub.stop();
  }

  @Override
  public boolean isFinished() {
    return indexSub.getOpticalSwitch();
  }
}
