package frc.robot.commands.CrispyPositionCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorCommands.ElevatorRestingPositionCmd;
import frc.robot.commands.PivotCommands.RunToTopLim;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class DownPosition extends ParallelCommandGroup {

  public DownPosition(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem) {
    
    addCommands(
      new RunToTopLim(pivotSubsystem), 

      new ElevatorRestingPositionCmd(elevatorSubsystem)
    );
  }
}
