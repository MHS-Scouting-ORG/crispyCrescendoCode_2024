package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.UnderIntakeSubsystem;

public class DeliverCmd extends Command {

  UnderIntakeSubsystem intakeSubs;

  // Runs the under the bumper intake to transfer the notes to the indexer

  public DeliverCmd(UnderIntakeSubsystem newIntakeSubs) {
    intakeSubs = newIntakeSubs;
    addRequirements(intakeSubs);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    intakeSubs.intake(0.6);
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubs.stopIntake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
