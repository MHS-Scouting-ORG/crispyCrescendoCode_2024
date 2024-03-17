package frc.robot.commands.ShindexerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeShooterCommand extends Command {

  private ShooterSubsystem shoot_Sub;
  private IndexerSubsystem index_Sub;

  public IntakeShooterCommand(ShooterSubsystem shooterSub, IndexerSubsystem indexer_Sub) {
    shoot_Sub = shooterSub;
    index_Sub = indexer_Sub;
    addRequirements(shoot_Sub);
    addRequirements(index_Sub);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shoot_Sub.shooter(-0.4);
    index_Sub.index(-0.5);
  }

  @Override
  public void end(boolean interrupted) {
    // if(index_Sub.getOpticalSwitch()){
    //   while(!index_Sub.getOpticalSwitch()){
    //     shoot_Sub.shooter(-0.4);
    //     index_Sub.index(-0.5);
    //   }
    // }
    
    shoot_Sub.stop();
    index_Sub.stop();
  }

  @Override
  public boolean isFinished() {
    return index_Sub.getOpticalSwitch();
  }
}