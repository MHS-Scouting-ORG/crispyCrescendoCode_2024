
package frc.robot.commands.CrispyPositionCommands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorCommands.ElevatorRestingPositionCmd;
import frc.robot.commands.ElevatorCommands.ElevatorToTransferCmd;
import frc.robot.commands.IntakeCommands.DeliverCmd;
import frc.robot.commands.ShindexerCommands.IndexerCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.UnderIntakeSubsystem;
import frc.robot.commands.ElevatorCommands.ElevatorToTopCmd;

public class AutomaticPickup extends SequentialCommandGroup {

  public AutomaticPickup(UnderIntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem, IndexerSubsystem indexerSubsystem) {
    addCommands(

      new ElevatorToTopCmd(elevatorSubsystem), //goes to mid position to pick up note from indexer 

      new FeedToIndexer(indexerSubsystem, intakeSubsystem), //feeds note from intake to indexer 

      new ElevatorRestingPositionCmd(elevatorSubsystem) 
      
    );
  }
}
