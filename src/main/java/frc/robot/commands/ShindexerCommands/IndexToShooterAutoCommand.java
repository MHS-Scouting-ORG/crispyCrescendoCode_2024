package frc.robot.commands.ShindexerCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShindexerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IndexToShooterAutoCommand extends Command {

  private ShooterSubsystem shootSub;
  private IndexerSubsystem indexSub;
  private Timer timer;

  public IndexToShooterAutoCommand(ShooterSubsystem sSub, IndexerSubsystem iSub) {
    shootSub = sSub;
    indexSub = iSub;
    timer = new Timer();
    addRequirements(shootSub);
    addRequirements(indexSub);
   }

 
  @Override
  public void initialize() {
    //shootSub.resetEncoder();
    indexSub.resetIndexerEnc();
    timer.reset();
    timer.start();
  }


  @Override
  public void execute() {
    // change statement to check if shooter rpm < specified speed
    if(shootSub.getRPM() > ShindexerConstants.RPM_SPEED_LIMIT){
      indexSub.index(ShindexerConstants.INDEXER_SPEED);
      shootSub.shooter(ShindexerConstants.SHOOTER_SPEED);
      
    } else {
      shootSub.shooter(ShindexerConstants.SHOOTER_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {
    indexSub.stop();
    shootSub.stop();
  }
 
  @Override
  public boolean isFinished() {
    //return shootSub.getRPM() > ShindexerConstants.RPM_SPEED_LIMIT + 200;
    return timer.get() >= 2;
    //return false;
  }

}