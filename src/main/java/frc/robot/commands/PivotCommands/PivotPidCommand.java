// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PivotCommands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class PivotPidCommand extends Command {
 
  PivotSubsystem pivotSub;
  double setpoint; 
  double offset = 5;

  public PivotPidCommand(PivotSubsystem pivotSubs, double setpoint){
    pivotSub = pivotSubs;
    this.setpoint = setpoint;

    addRequirements(pivotSub);
  }
   
  @Override
  public void initialize(){
    pivotSub.init();
    pivotSub.enablePid();
    //pivotSub.disablePid();
  }

  @Override
  public void execute(){
   SmartDashboard.putString("CURRENT CMD", getName());
    pivotSub.changeSetpoint(setpoint);
    SmartDashboard.putNumber("[P] CMD SETPT", setpoint); 
    //SmartDashboard.putNumber("setpoint", setpoint);
  }

  @Override
  public void end(boolean interrupted){
    SmartDashboard.putString("CURRENT CMD", "NONE");
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
