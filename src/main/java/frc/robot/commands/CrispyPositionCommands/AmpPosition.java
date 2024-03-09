package frc.robot.commands.CrispyPositionCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorCommands.ElevatorToTopCmd;
import frc.robot.commands.PivotCommands.PivotPidCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.UnderIntakeSubsystem;

public class AmpPosition extends SequentialCommandGroup {

  public AmpPosition(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem, IndexerSubsystem indexerSubsystem, UnderIntakeSubsystem intakeSubsystem) {
    addCommands(
      new ElevatorToTopCmd(elevatorSubsystem), 

      new PivotPidCommand(pivotSubsystem, 43), 

      new FeedToIndexer(indexerSubsystem, intakeSubsystem)
    );
  }
}
