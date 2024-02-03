package frc.robot.commands.ShindexerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {

  private ShooterSubsystem shootSub;
  private IndexerSubsystem indexSub;

  public ShooterCommand(ShooterSubsystem sSub, IndexerSubsystem iSub) {
    shootSub = sSub;
    indexSub = iSub;
    addRequirements(shootSub);
    addRequirements(indexSub);
   }

 
  @Override
  public void initialize() {
    // shootSub.resetEncoder();
  }


  @Override
  public void execute() {
    // change statement to check if shooter rpm < specified speed
    if(shootSub.getEncoder() < 100){
      shootSub.shooter(.5);
    }
    else{
      indexSub.index();
    }
  }

  
  @Override
  public void end(boolean interrupted) {
    // shootSub.resetEncoder();
    shootSub.stop();
    indexSub.stop();
  }
 
  @Override
  public boolean isFinished() {
    if(shootSub.getEncoder() > 100){
      return true;
    }
    return false;
  }

}