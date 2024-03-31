package frc.robot.commands.ShindexerCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShindexerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IndexAuto extends Command {

  private IndexerSubsystem indexSubsystem;
  ShooterSubsystem shooterSubsystem; 
  private Timer timer; 

  public IndexAuto(IndexerSubsystem indexSubsystem, ShooterSubsystem shooterSubsystem) {
    this.indexSubsystem = indexSubsystem;
    this.shooterSubsystem = shooterSubsystem; 
    timer = new Timer();

    addRequirements(indexSubsystem);
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    indexSubsystem.index(ShindexerConstants.INDEXER_SPEED);

    if(shooterSubsystem.getRPM() > ShindexerConstants.MAX_RPM * 0.9){
      indexSubsystem.index(ShindexerConstants.INDEXER_SPEED);
      shooterSubsystem.shooter(0.95);
      
    } else {
      shooterSubsystem.shooter(0.95);
    }
  }

  @Override
  public void end(boolean interrupted) {
    indexSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.get() > 1; 
  }
}