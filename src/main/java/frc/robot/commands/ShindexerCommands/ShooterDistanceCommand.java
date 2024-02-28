package frc.robot.commands.ShindexerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterDistanceCommand extends Command {
  
  private ShooterSubsystem shoot_Sub;

  public ShooterDistanceCommand(ShooterSubsystem shooter_Sub) {
    shoot_Sub = shooter_Sub;
    addRequirements(shoot_Sub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shoot_Sub.shootToSpeaker(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoot_Sub.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
