package frc.robot.commands.ShindexerCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShindexerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IndexToShooterAmpCommand extends Command {

  private ShooterSubsystem shootSub;
  private IndexerSubsystem indexSub;
  private Timer timer;

  public IndexToShooterAmpCommand(ShooterSubsystem sSub, IndexerSubsystem iSub) {
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
    if(shootSub.getRPM() > ShindexerConstants.RPM_AMP_SPEED_LIMIT){
      indexSub.index(0.2);
      shootSub.shooter(0.2);
      
    } else {
      shootSub.shooter(0.2);
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