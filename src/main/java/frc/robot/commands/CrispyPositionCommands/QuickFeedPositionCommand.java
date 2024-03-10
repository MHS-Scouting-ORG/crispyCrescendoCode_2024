package frc.robot.commands.CrispyPositionCommands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class QuickFeedPositionCommand extends Command {
  ElevatorSubsystem elevatorSubsystem;
  PivotSubsystem pivotSubsystem;
  public QuickFeedPositionCommand(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    addRequirements(elevatorSubsystem);
    addRequirements(pivotSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    elevatorSubsystem.setSetpoint(110);
    if(elevatorSubsystem.getEnc() >= 80){
      pivotSubsystem.changeSetpoint(45);
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.elevStop();
    pivotSubsystem.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.isAtSetpoint() && pivotSubsystem.atSetpoint();
  }
}
