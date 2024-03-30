package frc.robot.commands.CrispyPositionCommands;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorCommands.ElevatorToTransferCmd;
import frc.robot.commands.IntakeCommands.IntakeCmd;
import frc.robot.commands.PivotCommands.PivotPidCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.UnderIntakeSubsystem;

public class FeedPosition extends SequentialCommandGroup {

  public FeedPosition(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem, IndexerSubsystem indexerSubsystem, UnderIntakeSubsystem intakeSubsystem) {

    addCommands(
      new PivotPidCommand(pivotSubsystem, 30), 

      new FeedToIndexer(indexerSubsystem, intakeSubsystem)
    );
  }
}
