package frc.robot.commands.ShindexerCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShindexerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IndexToShooterAutoCommand extends Command {

  private ShooterSubsystem shootSub;
  private IndexerSubsystem indexSub;
  private double speed; 
  double seconds; 
  private Timer timer;

  public IndexToShooterAutoCommand(ShooterSubsystem sSub, IndexerSubsystem iSub, double speed, double seconds) {
    shootSub = sSub;
    indexSub = iSub;
    this.speed = speed;
    this.seconds = seconds; 
    timer = new Timer();
    addRequirements(shootSub);
    addRequirements(indexSub);
   }

  public IndexToShooterAutoCommand(ShooterSubsystem sSub, IndexerSubsystem iSub) {
    this(sSub, iSub, ShindexerConstants.TELEOP_SHOOTER_SPEED, 0.8);
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
    if(shootSub.getRPM() > ShindexerConstants.MAX_RPM * (speed)){
      indexSub.index(ShindexerConstants.INDEXER_SPEED);
      shootSub.shooter(speed);
      
    } else {
      shootSub.shooter(speed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    indexSub.stop();
    shootSub.stop();
  }
 
  @Override
  public boolean isFinished() {
    return timer.get() >= seconds;
    //return false;
  }

}