package frc.robot.commands.CrispyPositionCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorCommands.ElevatorToTransferCmd;
import frc.robot.commands.PivotCommands.PivotPidAlignCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ShootingPosition extends SequentialCommandGroup {

  public ShootingPosition(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem) {
    addCommands(
      new ElevatorToTransferCmd(elevatorSubsystem), 

      new PivotPidAlignCommand(pivotSubsystem)
    );
  }
}
