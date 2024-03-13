package frc.robot.commands.PivotCommands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class PivotPidCommand extends Command {
 
  PivotSubsystem pivotSub;
  double setpoint; 
  //double offset = 5;

  public PivotPidCommand(PivotSubsystem pivotSubs, double setpoint){
    pivotSub = pivotSubs;
    this.setpoint = setpoint;

    addRequirements(pivotSub);
  }
   
  @Override
  public void initialize(){
    pivotSub.init();
    pivotSub.enablePid();
  }

  @Override
  public void execute(){
    pivotSub.changeSetpoint(setpoint);
  }

  @Override
  public void end(boolean interrupted){
    pivotSub.stopMotor();
  }

  @Override
  public boolean isFinished() {
    if(pivotSub.atSetpoint()){
      return true;
    }
    return false;
  }
}
