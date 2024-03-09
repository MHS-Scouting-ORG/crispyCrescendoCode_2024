package frc.robot.commands.PivotCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class RunToTopLim extends Command {
  PivotSubsystem pivotSubs;
  double offset = 5;

  
  public RunToTopLim(PivotSubsystem pivotSubsystem){
    pivotSubs = pivotSubsystem;
    addRequirements(pivotSubs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    pivotSubs.init();
    pivotSubs.enablePid();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    SmartDashboard.putString("CURRENT COMMAND", getName()); 
    pivotSubs.changeSetpoint(60 + offset - 2);

  }

  @Override
  public void end(boolean interrupted){
    SmartDashboard.putString("CURRENT COMMAND", "NONE");
    pivotSubs.stopMotor();
  }

  @Override
  public boolean isFinished() {
    if(pivotSubs.topLimitSwitchPressed() || pivotSubs.atSetpoint()){
      return true;
    }
    return false;
  }
}
