package frc.robot.commands.CrispyPositionCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PivotCommands.PivotPidAlignCommand;
import frc.robot.commands.ShindexerCommands.IndexToShooterAutoCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class AlignPivotShoot extends SequentialCommandGroup {

  public AlignPivotShoot(PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem) {
    addCommands(
      new PivotPidAlignCommand(pivotSubsystem), 

      new IndexToShooterAutoCommand(shooterSubsystem, indexerSubsystem)
    );
  }
}
