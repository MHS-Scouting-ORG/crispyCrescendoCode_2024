package frc.robot.commands.CrispyPositionCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorCommands.ElevatorToTopCmd;
import frc.robot.commands.ElevatorCommands.ElevatorToTransferCmd;
import frc.robot.commands.PivotCommands.PivotPidCommand;
import frc.robot.commands.ShindexerCommands.IndexToShooterAmpCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.UnderIntakeSubsystem;

public class AmpPosition extends SequentialCommandGroup {

  public AmpPosition(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem, IndexerSubsystem indexerSubsystem, UnderIntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    addCommands(
      new ParallelCommandGroup(
        new ElevatorToTopCmd(elevatorSubsystem), 
        new PivotPidCommand(pivotSubsystem, 45), 
        new InstantCommand(() -> shooterSubsystem.shooter(0.15))
      ), 

      new IndexToShooterAmpCommand(shooterSubsystem, indexerSubsystem)



    );
  }
}
