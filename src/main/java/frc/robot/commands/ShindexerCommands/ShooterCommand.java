package frc.robot.commands.ShindexerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShindexerConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
  
  private ShooterSubsystem shoot_Sub;

  public ShooterCommand(ShooterSubsystem shooterSub) {
    shoot_Sub = shooterSub;
    addRequirements(shoot_Sub);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shoot_Sub.shooter(ShindexerConstants.SHOOTER_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    shoot_Sub.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
